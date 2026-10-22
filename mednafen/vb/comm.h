#ifndef __VB_COMM_H
#define __VB_COMM_H

#include <libretro.h>

#include "../mednafen-types.h"

#include "../state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The communication port (EXT.): CCR, CCSR, CDTR and CDRR at 0x02000000-0C.
 * See comm.c. */

/* Ask the frontend for its link bus. Once, from retro_init. */
void COMM_Init(retro_environment_t env);

/* Join and leave the bus, around a loaded game. With no bus, or nothing cabled
 * to this unit, the port is a Virtual Boy with nothing in its EXT. socket. */
void COMM_Start(void);
void COMM_Stop(void);

void COMM_Power(void) MDFN_COLD;

uint8 COMM_Read(const v810_timestamp_t timestamp, uint32 A);
void COMM_Write(const v810_timestamp_t timestamp, uint32 A, uint8 V);

v810_timestamp_t COMM_Update(const v810_timestamp_t timestamp);

/* The CPU's timestamp is about to go back to zero, having reached `timestamp`. */
void COMM_ResetTS(const v810_timestamp_t timestamp);

int COMM_StateAction(StateMem *sm, int load, int data_only);

#ifdef __cplusplus
}
#endif

#endif
