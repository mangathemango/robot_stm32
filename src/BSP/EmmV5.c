#include "EmmV5.h"

/* ---------------------------------------------------------------
 *  Emm_V5_Reset_CurPos_To_Zero
 *  Set current position as zero
 * --------------------------------------------------------------- */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr; // address
    cmd[1] = 0x0A; // function code
    cmd[2] = 0x6D; // auxiliary code
    cmd[3] = 0x6B; // checksum

    can_SendCmd(cmd, 4);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Reset_Clog_Pro
 *  Clear stall protection
 * --------------------------------------------------------------- */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr; // address
    cmd[1] = 0x0E; // function code
    cmd[2] = 0x52; // auxiliary code
    cmd[3] = 0x6B; // checksum

    can_SendCmd(cmd, 4);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Read_Sys_Params
 *  Read a system parameter
 * --------------------------------------------------------------- */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    cmd[i++] = addr; // address

    switch (s)
    {
    case S_VER:
        cmd[i++] = 0x1F;
        break;
    case S_RL:
        cmd[i++] = 0x20;
        break;
    case S_PID:
        cmd[i++] = 0x21;
        break;
    case S_VBUS:
        cmd[i++] = 0x24;
        break;
    case S_CPHA:
        cmd[i++] = 0x27;
        break;
    case S_ENCL:
        cmd[i++] = 0x31;
        break;
    case S_TPOS:
        cmd[i++] = 0x33;
        break;
    case S_VEL:
        cmd[i++] = 0x35;
        break;
    case S_CPOS:
        cmd[i++] = 0x36;
        break;
    case S_PERR:
        cmd[i++] = 0x37;
        break;
    case S_FLAG:
        cmd[i++] = 0x3A;
        break;
    case S_ORG:
        cmd[i++] = 0x3B;
        break;
    case S_Conf:
        cmd[i++] = 0x42;
        cmd[i++] = 0x6C;
        break;
    case S_State:
        cmd[i++] = 0x43;
        cmd[i++] = 0x7A;
        break;
    default:
        break;
    }

    cmd[i++] = 0x6B; // checksum

    can_SendCmd(cmd, i);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Modify_Ctrl_Mode
 *  Switch open-loop / closed-loop control mode
 *
 *  ctrl_mode:
 *    0 = disable pulse input pin
 *    1 = open-loop
 *    2 = closed-loop
 *    3 = En pin → multi-turn limit switch, Dir pin → position-reached output
 * --------------------------------------------------------------- */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;         // address
    cmd[1] = 0x46;         // function code
    cmd[2] = 0x69;         // auxiliary code
    cmd[3] = (uint8_t)svF; // store flag
    cmd[4] = ctrl_mode;    // control mode
    cmd[5] = 0x6B;         // checksum

    can_SendCmd(cmd, 6);
}

/* ---------------------------------------------------------------
 *  Emm_V5_En_Control
 *  Enable / disable the motor
 * --------------------------------------------------------------- */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;           // address
    cmd[1] = 0xF3;           // function code
    cmd[2] = 0xAB;           // auxiliary code
    cmd[3] = (uint8_t)state; // enable state: true = on, false = off
    cmd[4] = (uint8_t)snF;   // multi-motor sync flag
    cmd[5] = 0x6B;           // checksum

    can_SendCmd(cmd, 6);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Vel_Control
 *  Velocity mode
 *
 *  addr : motor address (e.g. 0x01 … 0x04 for 4 motors)
 *  dir  : direction — MOTOR_DIR_CW (0) or MOTOR_DIR_CCW (1)
 *  vel  : speed in RPM, range 0–5000
 *  acc  : acceleration 0–255  (0 = instant start)
 *  snF  : multi-motor sync flag
 * --------------------------------------------------------------- */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;                  // address
    cmd[1] = 0xF6;                  // function code
    cmd[2] = dir;                   // direction
    cmd[3] = (uint8_t)(vel >> 8);   // speed high byte
    cmd[4] = (uint8_t)(vel & 0xFF); // speed low byte
    cmd[5] = acc;                   // acceleration
    cmd[6] = (uint8_t)snF;          // sync flag
    cmd[7] = 0x6B;                  // checksum

    can_SendCmd(cmd, 8);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Pos_Control
 *  Position mode
 *
 *  addr : motor address
 *  dir  : direction
 *  vel  : speed in RPM
 *  acc  : acceleration 0–255  (0 = instant start)
 *  clk  : pulse count (steps), range 0–(2^32 - 1)
 *  raF  : false = relative move, true = absolute move
 *  snF  : multi-motor sync flag
 *
 *  This command is 13 bytes — can_SendCmd will split it into
 *  two CAN frames automatically (packet 0 + packet 1).
 * --------------------------------------------------------------- */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                        uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;                  // address
    cmd[1] = 0xFD;                  // function code
    cmd[2] = dir;                   // direction
    cmd[3] = (uint8_t)(vel >> 8);   // speed high byte
    cmd[4] = (uint8_t)(vel & 0xFF); // speed low byte
    cmd[5] = acc;                   // acceleration
    cmd[6] = (uint8_t)(clk >> 24);  // pulse count bits 31-24
    cmd[7] = (uint8_t)(clk >> 16);  // pulse count bits 23-16
    cmd[8] = (uint8_t)(clk >> 8);   // pulse count bits 15-8
    cmd[9] = (uint8_t)(clk & 0xFF); // pulse count bits 7-0
    cmd[10] = (uint8_t)raF;         // relative / absolute flag
    cmd[11] = (uint8_t)snF;         // sync flag
    cmd[12] = 0x6B;                 // checksum

    can_SendCmd(cmd, 13); // will be split: packet0 (8B) + packet1 (5B)
}

/* ---------------------------------------------------------------
 *  Emm_V5_Stop_Now
 *  Immediate stop (works in all control modes)
 * --------------------------------------------------------------- */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;         // address
    cmd[1] = 0xFE;         // function code
    cmd[2] = 0x98;         // auxiliary code
    cmd[3] = (uint8_t)snF; // sync flag
    cmd[4] = 0x6B;         // checksum

    can_SendCmd(cmd, 5);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Synchronous_motion
 *  Trigger all motors flagged with snF=true to move simultaneously
 * --------------------------------------------------------------- */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr; // address
    cmd[1] = 0xFF; // function code
    cmd[2] = 0x66; // auxiliary code
    cmd[3] = 0x6B; // checksum

    can_SendCmd(cmd, 4);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Origin_Set_O
 *  Set the zero (home) position for single-turn homing
 * --------------------------------------------------------------- */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;         // address
    cmd[1] = 0x93;         // function code
    cmd[2] = 0x88;         // auxiliary code
    cmd[3] = (uint8_t)svF; // store flag
    cmd[4] = 0x6B;         // checksum

    can_SendCmd(cmd, 5);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Origin_Modify_Params
 *  Modify homing parameters
 *
 *  o_mode : 0=single-turn nearest, 1=single-turn directional,
 *            2=multi-turn collision, 3=multi-turn limit-switch
 *  o_dir  : homing direction (0=CW, other=CCW)
 *  o_vel  : homing speed (RPM)
 *  o_tm   : homing timeout (ms)
 *  sl_vel : stall-detection speed (RPM)
 *  sl_ma  : stall-detection current (mA)
 *  sl_ms  : stall-detection time (ms)
 *  potF   : power-on auto-home flag
 *
 *  This is a 20-byte command — split into 3 CAN frames.
 * --------------------------------------------------------------- */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir,
                                 uint16_t o_vel, uint32_t o_tm,
                                 uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    cmd[0] = addr;                      // address
    cmd[1] = 0x4C;                      // function code
    cmd[2] = 0xAE;                      // auxiliary code
    cmd[3] = (uint8_t)svF;              // store flag
    cmd[4] = o_mode;                    // homing mode
    cmd[5] = o_dir;                     // homing direction
    cmd[6] = (uint8_t)(o_vel >> 8);     // homing speed high byte
    cmd[7] = (uint8_t)(o_vel & 0xFF);   // homing speed low byte
    cmd[8] = (uint8_t)(o_tm >> 24);     // timeout bits 31-24
    cmd[9] = (uint8_t)(o_tm >> 16);     // timeout bits 23-16
    cmd[10] = (uint8_t)(o_tm >> 8);     // timeout bits 15-8
    cmd[11] = (uint8_t)(o_tm & 0xFF);   // timeout bits 7-0
    cmd[12] = (uint8_t)(sl_vel >> 8);   // stall speed high byte
    cmd[13] = (uint8_t)(sl_vel & 0xFF); // stall speed low byte
    cmd[14] = (uint8_t)(sl_ma >> 8);    // stall current high byte
    cmd[15] = (uint8_t)(sl_ma & 0xFF);  // stall current low byte
    cmd[16] = (uint8_t)(sl_ms >> 8);    // stall time high byte
    cmd[17] = (uint8_t)(sl_ms & 0xFF);  // stall time low byte
    cmd[18] = (uint8_t)potF;            // power-on auto-home flag
    cmd[19] = 0x6B;                     // checksum

    can_SendCmd(cmd, 20); // split: packet0 + packet1 + packet2
}

/* ---------------------------------------------------------------
 *  Emm_V5_Origin_Trigger_Return
 *  Trigger homing
 * --------------------------------------------------------------- */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;         // address
    cmd[1] = 0x9A;         // function code
    cmd[2] = o_mode;       // homing mode
    cmd[3] = (uint8_t)snF; // sync flag
    cmd[4] = 0x6B;         // checksum

    can_SendCmd(cmd, 5);
}

/* ---------------------------------------------------------------
 *  Emm_V5_Origin_Interrupt
 *  Force-stop and exit homing
 * --------------------------------------------------------------- */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr; // address
    cmd[1] = 0x9C; // function code
    cmd[2] = 0x48; // auxiliary code
    cmd[3] = 0x6B; // checksum

    can_SendCmd(cmd, 4);
}