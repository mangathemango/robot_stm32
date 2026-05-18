#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "can.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------
 *  Helper macro
 * --------------------------------------------------------------- */
#define ABS(x) ((x) > 0 ? (x) : -(x))

/* ---------------------------------------------------------------
 *  System parameter selector (used by Emm_V5_Read_Sys_Params)
 * --------------------------------------------------------------- */
typedef enum
{
    S_VER = 0,    /* Firmware version + hardware version */
    S_RL = 1,     /* Phase resistance and inductance      */
    S_PID = 2,    /* PID parameters                      */
    S_VBUS = 3,   /* Bus voltage                         */
    S_CPHA = 5,   /* Phase current                       */
    S_ENCL = 7,   /* Linearised encoder value            */
    S_TPOS = 8,   /* Target position angle               */
    S_VEL = 9,    /* Real-time speed                     */
    S_CPOS = 10,  /* Real-time position angle            */
    S_PERR = 11,  /* Position error angle                */
    S_FLAG = 13,  /* Enable / in-position / stall flags  */
    S_Conf = 14,  /* Driver configuration parameters     */
    S_State = 15, /* System state parameters             */
    S_ORG = 16,   /* Homing / homing-fail flags          */
} SysParams_t;

/* ---------------------------------------------------------------
 *  Motor direction
 * --------------------------------------------------------------- */
#define MOTOR_DIR_CW 0
#define MOTOR_DIR_CCW 1

/* ---------------------------------------------------------------
 *  Function prototypes
 * --------------------------------------------------------------- */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir,
                                 uint16_t o_vel, uint32_t o_tm,
                                 uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);

#endif /* __EMM_V5_H */