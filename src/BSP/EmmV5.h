#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "can.h"
#include <stdbool.h>
#include <stdint.h>

/* ================================================================
 *  Emm V5 — HAL CAN driver
 *  Rewritten from ZDT StdPeriph example for STM32 HAL
 * ================================================================ */

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* ---------------------------------------------------------------
 *  Direction constants
 * --------------------------------------------------------------- */
#define MOTOR_DIR_CW 0
#define MOTOR_DIR_CCW 1

#define TOP_LEFT_MOTOR 0x01
#define TOP_RIGHT_MOTOR 0x02
#define BACK_LEFT_MOTOR 0x03
#define BACK_RIGHT_MOTOR 0x04

#define VER_MOTOR 0x05
#define HOR_MOTOR 0x06

#define PULSES_PER_REV 3200
/* ---------------------------------------------------------------
 *  Fast-position move mode (Emm_V5_FastPos_SetParams)
 * --------------------------------------------------------------- */
#define MOVE_RELATIVE_PREV 0 // relative to last input target position
#define MOVE_ABSOLUTE 1      // absolute from coordinate zero
#define MOVE_RELATIVE_NOW 2  // relative to current real-time position

/* ---------------------------------------------------------------
 *  System parameter selector (Emm_V5_Read_Sys_Params)
 * --------------------------------------------------------------- */
typedef enum
{
    S_VER = 0,     /* Firmware + hardware version  (func 0x1F) */
    S_RL = 1,      /* Phase resistance + inductance (func 0x20) */
    S_PID = 2,     /* PID parameters               (func 0x21) */
    S_VBUS = 3,    /* Bus voltage (mV)             (func 0x24) */
    S_CPHA = 5,    /* Phase current (mA)           (func 0x27) */
    S_ENCL = 7,    /* Linearised encoder 0-65535   (func 0x31) */
    S_PULSES = 17, /* Input pulse count (same unit as clk)  (func 0x32) */
    S_TPOS = 8,    /* Target position angle        (func 0x33) */
    S_VEL = 9,     /* Real-time speed (RPM)        (func 0x35) */
    S_CPOS = 10,   /* Real-time position angle     (func 0x36) */
    S_PERR = 11,   /* Position error angle         (func 0x37) */
    S_FLAG = 13,   /* Motor status flags           (func 0x3A) */
    S_Conf = 14,   /* Driver config params         (func 0x42) */
    S_State = 15,  /* System state params          (func 0x43) */
    S_ORG = 16,    /* Homing status flags          (func 0x3B) */
} SysParams_t;

/* ---------------------------------------------------------------
 *  Parsed motor response struct
 *
 *  motorResp[1..4] is indexed by motor address.
 *  Call CAN_ParseResponse() whenever can.rxFrameFlag goes true.
 *
 *  Command ACK codes returned by write commands:
 *    0x02 = received OK
 *    0x9F = received OK but motor is stalled
 *    0xE2 = checksum error
 *    0xEE = unknown command
 * --------------------------------------------------------------- */
typedef struct
{
    uint8_t addr;     // motor address that sent this reply
    uint8_t funcCode; // function code of the reply
    uint8_t ack;      // raw ack byte (0x02 = OK)

    /* velocity (S_VEL, func 0x35) */
    int16_t velocity_rpm; // signed RPM

    /* real-time position (S_CPOS, func 0x36) */
    float position_deg;   // degrees (Emm: raw*360/65536 per rev)
    int8_t position_sign; // +1 or -1

    /* position error (S_PERR, func 0x37) */
    float pos_error_deg;

    /* bus voltage (S_VBUS, func 0x24) */
    uint32_t vbus_mv; // millivolts

    /* phase current (S_CPHA, func 0x27) */
    uint32_t current_ma; // milliamps

    /* linearised encoder (S_ENCL, func 0x31) */
    uint16_t encoder; // 0-65535 = 0-360°

    /* motor status flags (S_FLAG / func 0x3A) */
    bool enabled;       // Ens_TF : motor is enabled
    bool in_position;   // Prf_TF : reached target position
    bool stalled;       // Cgi_TF : motor is stalling
    bool stall_protect; // Cgp_TF : stall protection triggered
    bool power_lost;    // Oac_TF : power loss event recorded

    /* homing status flags (S_ORG / func 0x3B) */
    bool encoder_ok;   // Enc_Rdy
    bool calibrated;   // Cal_Rdy
    bool homing;       // Org_SF : currently homing
    bool home_failed;  // Org_CF : homing failed
    bool over_temp;    // Otp_TF
    bool over_current; // Ocp_TF

} Emm_Response_t;

/* motorResp[0] unused; [1]..[4] = motors 1-4 */
extern volatile Emm_Response_t motorResp[5];

/* ---------------------------------------------------------------
 *  Call after can.rxFrameFlag goes true — decodes the RX frame
 *  and populates the right motorResp[addr] entry.
 * --------------------------------------------------------------- */
void CAN_ParseResponse(void);

/* ================================================================
 *  Motion commands
 * ================================================================ */

/** Enable / disable motor.
 *  state: true=enable, false=disable   snF: sync flag */
void En_Control(uint8_t addr, bool state, bool snF);

/** Velocity mode.
 *  dir: MOTOR_DIR_CW / MOTOR_DIR_CCW
 *  vel: 0-3000 RPM
 *  acc: 0-255  (0 = instant start, higher = faster ramp)
 *  snF: false=run now, true=buffer for sync trigger */
void Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

/** Position mode.
 *  clk: pulse count — 3200 pulses = 1 revolution (16 microstep, 1.8° motor)
 *  raF: false=relative, true=absolute
 *  snF: sync flag */
void Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                 float revs, bool raF, bool snF);

/** Fast position — step 1: set speed/acc/mode once.
 *  move_mode: MOVE_RELATIVE_PREV / MOVE_ABSOLUTE / MOVE_RELATIVE_NOW */
void FastPos_SetParams(uint8_t addr, uint16_t vel, uint8_t acc,
                       uint8_t move_mode, bool snF);

/** Fast position — step 2: send only the pulse count each time (signed).
 *  Much faster for rapid position updates (6 fewer bytes per frame). */
void FastPos_Move(uint8_t addr, int32_t clk);

/** Immediate stop — works in all control modes. */
void Stop_Now(uint8_t addr, bool snF);

/** Trigger buffered sync motion. Use broadcast addr 0x00 to fire all motors. */
void Synchronous_motion(uint8_t addr);

/* ================================================================
 *  Read system parameters
 *  The motor replies asynchronously over CAN.
 *  Usage:
 *    Emm_V5_Read_Sys_Params(0x01, S_VEL);
 *    // wait for can.rxFrameFlag, then:
 *    CAN_ParseResponse();
 *    int16_t rpm = motorResp[1].velocity_rpm;
 * ================================================================ */
void Read_Sys_Params(uint8_t addr, SysParams_t s);

/* ================================================================
 *  Homing commands
 * ================================================================ */

/** Set current position as zero reference. */
void Reset_CurPos_To_Zero(uint8_t addr);

/** Store current position as the single-turn homing zero point. */
void Origin_Set_O(uint8_t addr, bool svF);

/** Trigger homing.
 *  o_mode: 0=single-turn nearest, 1=single-turn dir, 2=collision,
 *           3=limit-switch, 4=absolute zero, 5=last power-off position */
void Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);

/** Force-stop and exit homing. */
void Origin_Interrupt(uint8_t addr);

/** Modify homing parameters (see EmmV5.c for param details). */
void Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir,
                          uint16_t o_vel, uint32_t o_tm,
                          uint16_t sl_vel, uint16_t sl_ma,
                          uint16_t sl_ms, bool potF);

/* ================================================================
 *  Misc / config commands
 * ================================================================ */

/** Clear stall protection (motor locks after stall — call this to unlock). */
void Reset_Clog_Pro(uint8_t addr);

/** Switch open/closed loop mode.
 *  ctrl_mode: 0=disable pulse pin, 1=open-loop, 2=closed-loop */
void Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);

/** Change motor CAN address (takes effect immediately; save with svF=true). */
void Set_MotorID(uint8_t addr, bool svF, uint8_t new_id);

void Send_Velocities(int16_t vtl, int16_t vtr, int16_t vrl, int16_t vrr);

#endif /* __EMM_V5_H */