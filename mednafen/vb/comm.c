/* The Virtual Boy's communication port (EXT.), carried over the frontend's
 * link bus.
 *
 * Mednafen never emulated it: all four registers read 0 and a transfer never
 * ran, so no game saw the other end of a cable. Register behaviour here is the
 * Sacred Tech Scroll's (Guy Perfect), which agrees with Red Viper's.
 *
 * The bus is reached through RETRO_ENVIRONMENT_GET_LINK_INTERFACE, exactly as
 * libretro/RetroArch#19454 adds it to libretro.h; this fork's libretro-common
 * carries that block and there is no private header.
 *
 * WHAT THE PORT IS. Two units and a clocked 8-bit exchange, the Game Boy's
 * serial port with one more wire. A unit writes the byte it will send to CDTR
 * and sets C-Start. With its own clock selected (C-Clk-Sel 0) it shifts the
 * byte out at 50 kHz, 160 us in all; with the external clock selected it waits
 * for the other unit to do that. Either way both ends finish together, each
 * with the other's byte in CDRR, and each raises the communication interrupt
 * unless it is inhibited. Which end clocks is the GAME's choice, both units
 * being the same hardware.
 *
 * The extra wire is COMCNT, an open line either unit can pull low. CC-Rd reads
 * it live (this unit's CC-Wr AND the other's), and CC-Smp samples it when an
 * exchange completes. Games use it to agree who clocks before a byte moves.
 *
 * WHAT CROSSES THE BUS. Three messages, eight bytes each:
 *   MSG_CC     this unit's CC-Wr and CC-Sig, whenever they change
 *   MSG_CLOCK  "I am clocking a byte out, starting at this tick", with the byte
 *   MSG_REPLY  the answer to a clock: the byte that came back, and whether this
 *              unit was waiting for one at all
 * The unit that was clocked decides whether the exchange happened, so the two
 * ends can never disagree about it, and the unit that clocked holds C-Stat
 * until it has been told. Both then complete on the same tick.
 *
 * WHEN. Every message is stamped no earlier than the horizon this unit last
 * published, which is the promise that lets the other unit run without asking,
 * and is acted on when the receiver's own clock reaches the stamp, never when
 * it happens to arrive. The horizon is short (a quarter of a byte) while the
 * port is in use and long (ten times that) while it is idle, so two units that
 * are not talking cost each other fifty rendezvous a frame instead of five
 * hundred. The first byte after a silence leaves up to an idle horizon late;
 * no protocol notices 400 us. While the port is in use the stamp is a FIXED
 * distance ahead, so the other unit sees this one's own timing, 40 us later:
 * Mario's Tennis signals on COMCNT with pulses eight cycles wide and the other
 * unit polls for them, which only works if they arrive eight cycles wide. */

#include <stdlib.h>
#include <string.h>

#include <libretro.h>

#include "vb.h"
#include "comm.h"

#include "../state_helpers.h"

extern retro_log_printf_t log_cb;

/* Wire name. A peer with any other id is never joined to this one. */
#define LINK_PROTOCOL "vb-comm-1"

/* The V810's 20 MHz, the unit every timestamp in this core counts in. */
#define LINK_CLOCK_RATE ((uint64_t)20000000)

/* Eight bits at 50 kHz. */
#define XFER_TICKS ((uint64_t)3200)

#define GRAIN_FINE ((uint64_t)800)
#define GRAIN_IDLE ((uint64_t)8000)

/* How long the port stays on the fine grain after it was last used. Long
 * enough to cover the gaps inside a packet, short enough that a game moving
 * one packet a frame spends most of the frame on the idle grain. */
#define FINE_HOLD (16 * XFER_TICKS)

/* How many more times a unit says where its COMCNT stands once it has heard
 * from the other one. See `heard`. */
#define ANNOUNCE_AGAIN 2

/* A clocked unit answers within its own horizon, and this unit cannot run past
 * that horizon without being handed the answer. Only a message the bus dropped
 * is still unanswered this long after the byte should have finished. */
#define REPLY_TIMEOUT (4 * GRAIN_IDLE)

#define MSG_SIZE  8
#define MSG_CC    1
#define MSG_CLOCK 2
#define MSG_REPLY 3

/* MSG_REPLY: the unit was waiting on the external clock and took the byte. */
#define MSG_FLAG_TAKEN 1

/* The COMCNT pair, as carried in every message. */
#define CC_WR  1
#define CC_SIG 2

enum
{
   XFER_IDLE = 0,
   /* C-Stat set, external clock selected: waiting to be clocked. */
   XFER_ARMED,
   /* Clocking a byte into an empty socket. */
   XFER_LOCAL,
   /* Clocking a byte down the cable; the other unit has not answered yet. */
   XFER_CLOCKED,
   /* The exchange is settled and completes at xfer_end. */
   XFER_RUNNING
};

/* A power of two. A game signals on COMCNT with pulses a few cycles wide, so a
 * grain's worth of edges can be waiting at once. */
#define PENDING_MAX 512
#define PENDING_MASK (PENDING_MAX - 1)

static const struct retro_link_interface *link_if;
static struct retro_link_interface link_storage;
static retro_link_port_t *link_port;
static bool anchored;
static unsigned peers;
static int self_id;
static bool trace;

/* This unit's place on the link timeline: link_base is where the CPU's
 * timestamp was last zero. The CPU's clock restarts every frame and after a
 * reset; this one never goes back. */
static uint64_t link_base;
static uint64_t now;
/* The tick of the next rendezvous, and the furthest horizon published so far.
 * Nothing this unit originates may be stamped before the second. */
static uint64_t limit;
static uint64_t promised;
static uint64_t fine_until;
/* Whether the other unit has said anything since the cable last moved. Seating
 * a lead rebuilds the bus, and a message sent to a unit that has not rejoined
 * its timeline is dropped on the way -- for as long as that unit takes to run
 * its first frame, which is the host's business and can be any length. So this
 * unit repeats its COMCNT state at every rendezvous until the other one is
 * heard from, and a couple of times more: software reads that line within
 * milliseconds of power-on to find out whether it is the second unit switched
 * on (VUEngine, Elevated Speed), and a state said once into nothing would leave
 * both believing they were first. */
static bool heard;
static unsigned announce_left;

/* CCR */
static uint8 c_int_inh, c_clk_sel, c_stat;
/* CCSR */
static uint8 cc_int_inh, cc_int_lev, cc_sig, cc_smp, cc_wr;
static uint8 cdtr, cdrr;
/* The two interrupt requests, acknowledged separately. */
static uint8 c_irq, cc_irq;

static uint8 xfer_state;
static uint64_t xfer_end;
static uint8 xfer_in;
/* Each end's COMCNT pair as the exchange began. Both ends sample CC-Smp from
 * the same four bits, so they agree on it as the two units on one wire do. */
static uint8 xfer_cc, xfer_peer_cc;

/* The other unit's COMCNT pair, as of the last message this unit's clock has
 * reached. A line nobody drives reads high. */
static uint8 peer_cc;
static uint8 said_cc;
static bool said;

/* The inbox, a ring: what the other unit has said and this unit's clock has not
 * reached yet. pend_ctl counts the clocks and replies among it, which need the
 * CPU stopped at their tick; a COMCNT edge only has to be in place by the next
 * time the line is read. */
static uint64_t pend_tick[PENDING_MAX];
static uint8 pend_type[PENDING_MAX];
static uint8 pend_flags[PENDING_MAX];
static uint8 pend_data[PENDING_MAX];
static uint8 pend_cc[PENDING_MAX];
static unsigned pend_head, pend_count, pend_ctl;

static unsigned long exchanged, unanswered;

/* WARN: a frontend that filters its log still shows a cable going in. */
#define SAY(...) do { if (log_cb) log_cb(RETRO_LOG_WARN, __VA_ARGS__); } while (0)

/* VB_COMM_TRACE in the environment: every register access (R, W), every byte
 * that lands in CDRR (D) and every CC-Smp sample (S), stamped with the unit and
 * its link clock, runs of the same one folded. How a game's protocol is read. */
static char trace_op;
static uint32 trace_addr;
static uint8 trace_val;
static unsigned long trace_run;
static uint64_t trace_first;

static void trace_io(char op, uint32 A, uint8 V)
{
   if (op == trace_op && A == trace_addr && V == trace_val)
   {
      trace_run++;
      return;
   }
   if (trace_run > 0)
      SAY("[vb-comm] u%d t=%lu %c %02x %02x x%lu\n", self_id,
          (unsigned long)(trace_first & 0xFFFFFFFF), trace_op, (unsigned)trace_addr,
          (unsigned)trace_val, trace_run);
   trace_op = op;
   trace_addr = A;
   trace_val = V;
   trace_run = 1;
   trace_first = now;
}

static bool round_number(unsigned long n)
{
   unsigned long p = 1;
   while (p < n)
      p *= 10;
   return p == n;
}

static bool cabled(void)
{
   return link_port && peers == 2;
}

static uint8 own_cc(void)
{
   return (cc_wr ? CC_WR : 0) | (cc_sig ? CC_SIG : 0);
}

static void irq(void)
{
   VBIRQ_Assert(VBIRQ_SOURCE_COMM, c_irq || cc_irq);
}

static void touch(void)
{
   fine_until = now + FINE_HOLD;
}

static uint64_t grain(void)
{
   return (now < fine_until || xfer_state != XFER_IDLE) ? GRAIN_FINE : GRAIN_IDLE;
}

/* The tick to stamp something this unit originates now. */
static uint64_t stamp(void)
{
   /* The first rendezvous anchors the origin the bus measures ticks from; a
    * message sent before it lands in the peer's far future. Asking for where
    * this unit already stands is granted as soon as the peer has got there. */
   if (!anchored)
   {
      uint64_t safe = now + grain();
      if (safe < promised)
         safe = promised;
      link_if->advance(link_port, now, safe, now, NULL);
      promised = safe;
      anchored = true;
   }
   /* A fixed distance ahead while the port is in use, so what the other unit
    * sees is this unit's own timing, later: a COMCNT pulse eight cycles wide
    * arrives eight cycles wide. */
   return promised > now + GRAIN_FINE ? promised : now + GRAIN_FINE;
}

static void send_msg(uint8 type, uint8 flags, uint8 data, uint8 cc, uint64_t tick)
{
   uint8_t msg[MSG_SIZE];

   memset(msg, 0, sizeof(msg));
   msg[0] = type;
   msg[1] = flags;
   msg[2] = data;
   msg[3] = cc;
   link_if->send(link_port, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

static void say_cc(bool again)
{
   uint8 cc = own_cc();

   if (!cabled())
      return;
   if (said && cc == said_cc && !again)
      return;
   said = true;
   said_cc = cc;
   send_msg(MSG_CC, 0, 0, cc, stamp());
}

static void complete(void)
{
   uint8 mine, theirs;

   if (xfer_state == XFER_LOCAL)
   {
      /* Nothing in the socket: the line reads as this unit alone drives it. */
      mine = theirs = own_cc();
   }
   else
   {
      mine = xfer_cc;
      theirs = xfer_peer_cc;
   }

   cdrr = xfer_in;
   c_stat = 0;
   xfer_state = XFER_IDLE;

   if (!c_int_inh)
      c_irq = 1;

   cc_smp = ((mine & theirs) == (CC_WR | CC_SIG)) ? 1 : 0;
   if (!cc_int_inh && cc_smp == cc_int_lev)
      cc_irq = 1;
   irq();

   if (trace)
   {
      trace_io('D', 0x0C, cdrr);
      trace_io('S', 0x04, cc_smp);
   }
}

/* Begin shifting CDTR out on this unit's own clock. */
static void begin_clocking(void)
{
   touch();
   if (cabled())
   {
      uint64_t at = stamp();

      xfer_cc = own_cc();
      xfer_peer_cc = CC_WR | CC_SIG;
      xfer_in = 0xFF;
      send_msg(MSG_CLOCK, 0, cdtr, xfer_cc, at);
      xfer_state = XFER_CLOCKED;
      xfer_end = at + XFER_TICKS;
   }
   else
   {
      /* What an open socket shifts in is undefined; the idle line is high. */
      xfer_in = 0xFF;
      xfer_state = XFER_LOCAL;
      xfer_end = now + XFER_TICKS;
   }
}

/* The other unit never answered: finish the byte against an idle line. */
static void give_up(void)
{
   xfer_in = 0xFF;
   xfer_peer_cc = CC_WR | CC_SIG;
   xfer_state = XFER_RUNNING;
}

static void clocked(uint64_t tick, uint8 data, uint8 cc)
{
   uint64_t at = stamp();

   touch();
   if (xfer_state != XFER_ARMED)
   {
      /* Not waiting for a byte, so this unit takes no part in the exchange
       * and says so: the other end reads an idle line. */
      send_msg(MSG_REPLY, 0, 0xFF, own_cc(), at);
      unanswered++;
      if (round_number(unanswered))
         SAY("[vb-comm] %lu clock(s) from the other unit found this one not waiting\n",
             unanswered);
      return;
   }

   xfer_cc = own_cc();
   xfer_peer_cc = cc;
   xfer_in = data;
   send_msg(MSG_REPLY, MSG_FLAG_TAKEN, cdtr, xfer_cc, at);
   xfer_state = XFER_RUNNING;
   xfer_end = tick + XFER_TICKS;
   if (xfer_end < at)
      xfer_end = at;

   exchanged++;
   if (round_number(exchanged))
      SAY("[vb-comm] %lu byte(s) exchanged, the last as the clocked unit\n", exchanged);
}

static void replied(uint64_t tick, uint8 flags, uint8 data, uint8 cc)
{
   if (xfer_state != XFER_CLOCKED)
      return;

   xfer_peer_cc = cc;
   xfer_in = (flags & MSG_FLAG_TAKEN) ? data : 0xFF;
   xfer_state = XFER_RUNNING;
   if (xfer_end < tick)
      xfer_end = tick;

   if (flags & MSG_FLAG_TAKEN)
   {
      exchanged++;
      if (round_number(exchanged))
         SAY("[vb-comm] %lu byte(s) exchanged, the last as the clocking unit\n", exchanged);
   }
}

static void apply_due(void)
{
   while (pend_count > 0 && pend_tick[pend_head] <= now)
   {
      uint64_t tick = pend_tick[pend_head];
      uint8 type = pend_type[pend_head];
      uint8 flags = pend_flags[pend_head];
      uint8 data = pend_data[pend_head];
      uint8 cc = pend_cc[pend_head] & (CC_WR | CC_SIG);

      pend_head = (pend_head + 1) & PENDING_MASK;
      pend_count--;

      if (cc != peer_cc)
      {
         peer_cc = cc;
         touch();
      }
      if (type == MSG_CC)
         continue;
      pend_ctl--;
      if (type == MSG_CLOCK)
         clocked(tick, data, cc);
      else
         replied(tick, flags, data, cc);
   }
}

static void drop_pending(void)
{
   pend_head = pend_count = pend_ctl = 0;
}

static void pump(void)
{
   uint8_t buf[MSG_SIZE];
   uint64_t tick;
   unsigned from;
   size_t len = sizeof(buf);

   while (link_if->recv(link_port, &tick, &from, buf, &len))
   {
      if (len == MSG_SIZE && buf[0] >= MSG_CC && buf[0] <= MSG_REPLY)
      {
         heard = true;
         if (pend_count < PENDING_MAX)
         {
            unsigned at = (pend_head + pend_count) & PENDING_MASK;

            pend_tick[at] = tick;
            pend_type[at] = buf[0];
            pend_flags[at] = buf[1];
            pend_data[at] = buf[2];
            pend_cc[at] = buf[3];
            pend_count++;
            if (buf[0] != MSG_CC)
               pend_ctl++;
         }
         else
            SAY("[vb-comm] inbox full, message dropped\n");
      }
      len = sizeof(buf);
   }
}

static void refresh_peers(void)
{
   unsigned count = 0;
   int id = link_if->peers(link_port, &count);
   unsigned was = peers;

   /* The cable joins two units. A bus of more carries nothing. */
   peers = (id < 0 || count > 2) ? 0 : count;
   self_id = id;
   if (peers == was)
      return;

   /* What was in flight belonged to the old cable, and the other end has to
    * be told afresh where this one's COMCNT stands. */
   drop_pending();
   peer_cc = CC_WR | CC_SIG;
   said = false;
   anchored = false;
   heard = false;
   announce_left = ANNOUNCE_AGAIN;
   if (xfer_state == XFER_CLOCKED)
      give_up();
   SAY("[vb-comm] %u unit(s) on the wire, this one is %d\n", count, id);
}

static void rendezvous(void)
{
   uint64_t step, safe, grant;

   refresh_peers();

   step = grain();
   safe = now + step;
   if (safe < promised)
      safe = promised;
   /* No wake flags: the grant then depends on nothing but the ticks the two
    * units have published. */
   grant = link_if->advance(link_port, now, safe, now + step, NULL);
   promised = safe;
   anchored = true;
   pump();

   if (grant == RETRO_LINK_UNBOUNDED || grant <= now)
      limit = now + step;       /* uncabled: look again a grain from now */
   else
      limit = grant;

   if (cabled())
   {
      if (!heard || !said)
         say_cc(true);
      else if (announce_left > 0)
      {
         announce_left--;
         say_cc(true);
      }
   }
}

static void run_to(const v810_timestamp_t timestamp)
{
   uint64_t t = link_base + (uint64_t)(timestamp > 0 ? timestamp : 0);

   if (t > now)
      now = t;

   if (link_port)
   {
      if (now >= limit)
         rendezvous();
      apply_due();
      if (xfer_state == XFER_CLOCKED && now >= xfer_end + REPLY_TIMEOUT)
         give_up();
   }

   if ((xfer_state == XFER_LOCAL || xfer_state == XFER_RUNNING) && now >= xfer_end)
      complete();
}

static v810_timestamp_t next_event(const v810_timestamp_t timestamp)
{
   uint64_t next = (uint64_t)-1;
   uint64_t ahead;

   if (link_port)
   {
      next = limit;
      if (pend_ctl > 0)
      {
         unsigned i;

         for (i = 0; i < pend_count; i++)
         {
            unsigned at = (pend_head + i) & PENDING_MASK;

            if (pend_type[at] != MSG_CC)
            {
               if (pend_tick[at] < next)
                  next = pend_tick[at];
               break;
            }
         }
      }
      if (xfer_state == XFER_CLOCKED && xfer_end + REPLY_TIMEOUT < next)
         next = xfer_end + REPLY_TIMEOUT;
   }
   if ((xfer_state == XFER_LOCAL || xfer_state == XFER_RUNNING) && xfer_end < next)
      next = xfer_end;

   if (next == (uint64_t)-1)
      return VB_EVENT_NONONO;

   ahead = next > now ? next - now : 1;
   if (ahead > 0x1000000)
      ahead = 0x1000000;
   return timestamp + (v810_timestamp_t)ahead;
}

v810_timestamp_t COMM_Update(const v810_timestamp_t timestamp)
{
   run_to(timestamp);
   return next_event(timestamp);
}

void COMM_ResetTS(const v810_timestamp_t timestamp)
{
   link_base += (uint64_t)(timestamp > 0 ? timestamp : 0);
   if (now < link_base)
      now = link_base;
}

uint8 COMM_Read(const v810_timestamp_t timestamp, uint32 A)
{
   uint8 ret = 0;

   run_to(timestamp);

   switch (A & 0xFF)
   {
      case 0x00:
         /* C-Start reads back set; the unused bits read 1. */
         ret = 0x6D | (c_int_inh << 7) | (c_clk_sel << 4) | (c_stat << 1);
         break;
      case 0x04:
         ret = 0x60 | (cc_int_inh << 7) | (cc_int_lev << 4) | (cc_sig << 3) |
               (cc_smp << 2) | (cc_wr << 1);
         /* The line, live: low if either unit pulls it low. With nothing in
          * the socket only this unit is on it. */
         if (cc_wr && (!cabled() || (peer_cc & CC_WR)))
            ret |= 0x01;
         break;
      case 0x08:
         ret = cdtr;
         break;
      case 0x0C:
         ret = cdrr;
         break;
   }

   if (trace)
      trace_io('R', A & 0xFF, ret);

   VB_SetEvent(VB_EVENT_COMM, next_event(timestamp));
   return ret;
}

void COMM_Write(const v810_timestamp_t timestamp, uint32 A, uint8 V)
{
   run_to(timestamp);

   if (trace)
      trace_io('W', A & 0xFF, V);

   switch (A & 0xFF)
   {
      case 0x00:
      {
         uint8 clk = (V >> 4) & 1;

         c_int_inh = (V >> 7) & 1;
         if (c_int_inh)
            c_irq = 0;

         if (clk != c_clk_sel)
         {
            c_clk_sel = clk;
            /* Changing the clock source does not abort a transfer, it changes
             * what drives it: a unit that was waiting on the other's clock
             * starts shifting on its own, and one clocking an empty socket
             * stops until a clock arrives. A byte already on the cable is the
             * other unit's to finish. */
            if (xfer_state == XFER_ARMED && !clk)
               begin_clocking();
            else if (xfer_state == XFER_LOCAL && clk)
               xfer_state = XFER_ARMED;
         }

         if ((V & 0x04) && !c_stat)
         {
            c_stat = 1;
            if (c_clk_sel)
            {
               xfer_state = XFER_ARMED;
               touch();
            }
            else
               begin_clocking();
         }
         irq();
         break;
      }

      case 0x04:
         cc_int_inh = (V >> 7) & 1;
         cc_int_lev = (V >> 4) & 1;
         cc_sig = (V >> 3) & 1;
         cc_wr = (V >> 1) & 1;
         if (cc_int_inh)
            cc_irq = 0;
         irq();
         if (cabled() && (!said || own_cc() != said_cc))
         {
            touch();
            say_cc(false);
         }
         break;

      case 0x08:
         cdtr = V;
         break;
   }

   VB_SetEvent(VB_EVENT_COMM, next_event(timestamp));
}

void COMM_Power(void)
{
   /* The link clock carries on: the bus must never see it go backwards. What
    * the other unit has already sent stays queued and is answered by the unit
    * this one has become, or a byte it clocked would never finish. */
   c_int_inh = 0;
   c_clk_sel = 0;
   c_stat = 0;
   cc_int_inh = 1;
   cc_int_lev = 1;
   cc_sig = 1;
   cc_smp = 0;
   cc_wr = 1;
   cdtr = 0;
   cdrr = 0;
   c_irq = 0;
   cc_irq = 0;
   xfer_state = XFER_IDLE;
   xfer_in = 0xFF;
   xfer_cc = xfer_peer_cc = CC_WR | CC_SIG;
   said = false;

   VBIRQ_Assert(VBIRQ_SOURCE_COMM, false);
}

void COMM_Init(retro_environment_t env)
{
   const char *t = getenv("VB_COMM_TRACE");

   trace = t && *t && *t != '0';
   memset(&link_storage, 0, sizeof(link_storage));
   link_if = NULL;
   if (env(RETRO_ENVIRONMENT_GET_LINK_INTERFACE, &link_storage))
      link_if = &link_storage;
}

void COMM_Start(void)
{
   if (!link_if || link_port)
      return;
   link_port = link_if->attach(0, LINK_PROTOCOL, LINK_CLOCK_RATE);
   anchored = false;
   peers = 0;
   drop_pending();
   peer_cc = CC_WR | CC_SIG;
   said = false;
   limit = now;
   exchanged = unanswered = 0;
   /* Meet the bus before the first instruction runs. The game is loaded with
    * the CPU's clock at zero and nothing scheduled for this port, and left
    * alone the first rendezvous would be the guest's first touch of a link
    * register -- too late for it to have been told where the other unit's
    * COMCNT stands, which is the very thing software reads at power-on. */
   if (link_port)
      VB_SetEvent(VB_EVENT_COMM, 0);
   if (!link_port)
      SAY("[vb-comm] the frontend refused the port\n");
}

void COMM_Stop(void)
{
   if (trace)
   {
      trace_io('.', 0, 0);
      SAY("[vb-comm] u%d stopped: transfer state %u, C-Stat %u, clock %s, COMCNT out %u\n",
          self_id, (unsigned)xfer_state, (unsigned)c_stat, c_clk_sel ? "external" : "internal",
          (unsigned)cc_wr);
   }
   if (!link_port)
      return;
   link_if->detach(link_port);
   link_port = NULL;
   drop_pending();
   peers = 0;
   anchored = false;
}

/* The port's half of a savestate. The registers are saved as they stand.
 * Anything stamped on the bus clock is saved as how far it lies from `now`,
 * never as a tick: that clock belongs to the session and never goes back, so
 * a state is loaded into a bus that has moved on. What is still on the wire,
 * not yet pumped into the inbox, is the frontend's to put back. */
int COMM_StateAction(StateMem *sm, int load, int data_only)
{
   /* Static: the queue is too much for a stack that also holds the CPU's own
    * state action. Nothing here is reentrant. */
   static int64_t s_rel[PENDING_MAX];
   static uint8 s_type[PENDING_MAX], s_flags[PENDING_MAX];
   static uint8 s_data[PENDING_MAX], s_cc[PENDING_MAX];
   int64_t end_rel = (xfer_state != XFER_IDLE && xfer_state != XFER_ARMED) ?
                     (int64_t)(xfer_end - now) : 0;
   int64_t fine_rel = fine_until > now ? (int64_t)(fine_until - now) : 0;
   uint8 present = 1;
   uint8 said_b = said;
   uint32 count = pend_count;
   unsigned i;

   for (i = 0; i < PENDING_MAX; i++)
   {
      unsigned at = (pend_head + i) & PENDING_MASK;
      bool used = i < pend_count;

      s_rel[i] = used ? (int64_t)(pend_tick[at] - now) : 0;
      s_type[i] = used ? pend_type[at] : 0;
      s_flags[i] = used ? pend_flags[at] : 0;
      s_data[i] = used ? pend_data[at] : 0;
      s_cc[i] = used ? pend_cc[at] : 0;
   }

   {
      SFORMAT StateRegs[] =
      {
         SFVARN(present, "present"),
         SFVARN(c_int_inh, "c_int_inh"),
         SFVARN(c_clk_sel, "c_clk_sel"),
         SFVARN(c_stat, "c_stat"),
         SFVARN(cc_int_inh, "cc_int_inh"),
         SFVARN(cc_int_lev, "cc_int_lev"),
         SFVARN(cc_sig, "cc_sig"),
         SFVARN(cc_smp, "cc_smp"),
         SFVARN(cc_wr, "cc_wr"),
         SFVARN(cdtr, "cdtr"),
         SFVARN(cdrr, "cdrr"),
         SFVARN(c_irq, "c_irq"),
         SFVARN(cc_irq, "cc_irq"),
         SFVARN(xfer_state, "xfer_state"),
         SFVARN(xfer_in, "xfer_in"),
         SFVARN(xfer_cc, "xfer_cc"),
         SFVARN(xfer_peer_cc, "xfer_peer_cc"),
         SFVARN(end_rel, "xfer_end_rel"),
         SFVARN(fine_rel, "fine_rel"),
         SFVARN(peer_cc, "peer_cc"),
         SFVARN(said_cc, "said_cc"),
         SFVARN(said_b, "said"),
         SFVARN(count, "pend_count"),
         SFARRAYN(s_type, PENDING_MAX, "pend_type"),
         SFARRAYN(s_flags, PENDING_MAX, "pend_flags"),
         SFARRAYN(s_data, PENDING_MAX, "pend_data"),
         SFARRAYN(s_cc, PENDING_MAX, "pend_cc"),
         SFARRAY64N((uint64_t *)s_rel, PENDING_MAX, "pend_rel"),
         SFEND
      };

      if (load)
         present = 0;
      if (!MDFNSS_StateAction(sm, load, data_only, StateRegs, "COMM", true))
         return 0;
   }

   if (load)
   {
      drop_pending();
      if (!present)
      {
         /* A state from before the port existed: nothing was in its socket. */
         COMM_Power();
         return 1;
      }

      c_int_inh &= 1;
      c_clk_sel &= 1;
      c_stat &= 1;
      cc_int_inh &= 1;
      cc_int_lev &= 1;
      cc_sig &= 1;
      cc_smp &= 1;
      cc_wr &= 1;
      c_irq &= 1;
      cc_irq &= 1;
      if (xfer_state > XFER_RUNNING)
         xfer_state = XFER_IDLE;
      if (!c_stat)
         xfer_state = XFER_IDLE;
      else if (xfer_state == XFER_IDLE)
         c_stat = 0;
      if (count > PENDING_MAX)
         count = PENDING_MAX;
      for (i = 0; i < count; i++)
      {
         if (s_type[i] < MSG_CC || s_type[i] > MSG_REPLY)
            break;
         pend_tick[i] = now + s_rel[i];
         pend_type[i] = s_type[i];
         pend_flags[i] = s_flags[i];
         pend_data[i] = s_data[i];
         pend_cc[i] = s_cc[i];
         pend_count++;
         if (s_type[i] != MSG_CC)
            pend_ctl++;
      }
      xfer_end = now + end_rel;
      fine_until = now + (fine_rel > 0 ? (uint64_t)fine_rel : 0);
      said = said_b != 0;
      /* Meet the bus at once: it has not heard from this unit's new past. */
      limit = now;
   }
   return 1;
}
