#include "EmmV5.h"

/* ================================================================
 *  Global response table — indexed by motor address (1-4)
 * ================================================================ */
volatile Emm_Response_t motorResp[5] = {0};

/* ================================================================
 *  CAN_ParseResponse
 *
 *  Call this every time can.rxFrameFlag goes true.
 *  It reads can.rxFrame, decodes the payload based on the function
 *  code, and stores the result in motorResp[addr].
 *
 *  Extended ID format from motor:  high byte = addr, low byte = packet
 * ================================================================ */
void CAN_ParseResponse(void)
{
    /* Extract motor address from the extended CAN ID */
    uint8_t addr = (can.rxFrame.header.ExtId >> 8) & 0xFF;
    uint8_t funcCode = can.rxFrame.data[0];
    uint8_t *d = (uint8_t *)can.rxFrame.data;
    uint8_t dlc = can.rxFrame.header.DLC;

    if (addr == 0 || addr > 4)
        return; // ignore out-of-range

    motorResp[addr].addr = addr;
    motorResp[addr].funcCode = funcCode;

    switch (funcCode)
    {
    /* ---- velocity (func 0x35): sign + 2-byte RPM ---- */
    case 0x35:
    {
        if (dlc < 4)
            break;
        int8_t sign = (d[1] == 0x00) ? +1 : -1;
        uint16_t raw = ((uint16_t)d[2] << 8) | d[3];
        motorResp[addr].velocity_rpm = sign * (int16_t)raw;
        break;
    }

    /* ---- real-time position (func 0x36): sign + 4-byte value ---- */
    case 0x36:
    {
        if (dlc < 6)
            break;
        int8_t sign = (d[1] == 0x00) ? +1 : -1;
        uint32_t raw = ((uint32_t)d[2] << 24) | ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];
        /* Emm firmware: 0-65535 = one revolution 0-360° */
        uint16_t single_turn = (uint16_t)(raw & 0xFFFF);
        motorResp[addr].position_deg = sign * (single_turn * 360.0f / 65536.0f);
        motorResp[addr].position_sign = sign;
        break;
    }

    /* ---- position error (func 0x37): sign + 4-byte value ---- */
    case 0x37:
    {
        if (dlc < 6)
            break;
        int8_t sign = (d[1] == 0x00) ? +1 : -1;
        uint32_t raw = ((uint32_t)d[2] << 24) | ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];
        uint16_t single_turn = (uint16_t)(raw & 0xFFFF);
        motorResp[addr].pos_error_deg = sign * (single_turn * 360.0f / 65536.0f);
        break;
    }

    /* ---- bus voltage (func 0x24): 4-byte mV ---- */
    case 0x24:
    {
        if (dlc < 5)
            break;
        motorResp[addr].vbus_mv = ((uint32_t)d[1] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 8) | d[4];
        break;
    }

    /* ---- phase current (func 0x27): 4-byte mA ---- */
    case 0x27:
    {
        if (dlc < 5)
            break;
        motorResp[addr].current_ma = ((uint32_t)d[1] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 8) | d[4];
        break;
    }

    /* ---- linearised encoder (func 0x31): 2-byte 0-65535 ---- */
    case 0x31:
    {
        if (dlc < 3)
            break;
        motorResp[addr].encoder = ((uint16_t)d[1] << 8) | d[2];
        break;
    }

    /* ---- motor status flags (func 0x3A): 1-byte bitmask ---- */
    case 0x3A:
    {
        if (dlc < 2)
            break;
        uint8_t flags = d[1];
        motorResp[addr].enabled = (flags & 0x01) != 0;
        motorResp[addr].in_position = (flags & 0x02) != 0;
        motorResp[addr].stalled = (flags & 0x04) != 0;
        motorResp[addr].stall_protect = (flags & 0x08) != 0;
        motorResp[addr].power_lost = (flags & 0x80) != 0;
        break;
    }

    /* ---- homing status flags (func 0x3B): 1-byte bitmask ---- */
    case 0x3B:
    {
        if (dlc < 2)
            break;
        uint8_t flags = d[1];
        motorResp[addr].encoder_ok = (flags & 0x01) != 0;
        motorResp[addr].calibrated = (flags & 0x02) != 0;
        motorResp[addr].homing = (flags & 0x04) != 0;
        motorResp[addr].home_failed = (flags & 0x08) != 0;
        motorResp[addr].over_temp = (flags & 0x10) != 0;
        motorResp[addr].over_current = (flags & 0x20) != 0;
        break;
    }

    /* ---- all other write-command acks ---- */
    default:
    {
        /* d[1] = ack byte: 0x02=OK, 0x9F=stalled, 0xE2=chksum err, 0xEE=unknown */
        if (dlc >= 2)
            motorResp[addr].ack = d[1];
        break;
    }
    }
}

/* ================================================================
 *  Motion commands
 * ================================================================ */

void En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[8] = {0};
    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state;
    cmd[4] = (uint8_t)snF;
    cmd[5] = 0x6B;
    can_SendCmd(cmd, 6);
}

void Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8] = {0};
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel & 0xFF);
    cmd[5] = acc;
    cmd[6] = (uint8_t)snF;
    cmd[7] = 0x6B;
    can_SendCmd(cmd, 8);
}

void Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                 float revs, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    uint32_t clk = (uint32_t)(revs * PULSES_PER_REV);
    cmd[0] = addr;
    cmd[1] = 0xFD;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel & 0xFF);
    cmd[5] = acc;
    cmd[6] = (uint8_t)(clk >> 24);
    cmd[7] = (uint8_t)(clk >> 16);
    cmd[8] = (uint8_t)(clk >> 8);
    cmd[9] = (uint8_t)(clk & 0xFF);
    cmd[10] = (uint8_t)raF;
    cmd[11] = (uint8_t)snF;
    cmd[12] = 0x6B;

    can_SendCmd(cmd, 13); // auto-split: packet0 (8B) + packet1 (5B)
}

/* Fast position — step 1: set params once (func 0xF1) */
void FastPos_SetParams(uint8_t addr, uint16_t vel, uint8_t acc,
                       uint8_t move_mode, bool snF)
{
    uint8_t cmd[8] = {0};
    cmd[0] = addr;
    cmd[1] = 0xF1;
    cmd[2] = (uint8_t)(vel >> 8);
    cmd[3] = (uint8_t)(vel & 0xFF);
    cmd[4] = acc;
    cmd[5] = move_mode;
    cmd[6] = (uint8_t)snF;
    cmd[7] = 0x6B;
    can_SendCmd(cmd, 8);
}

/* Fast position — step 2: send signed pulse count only (func 0xFC)
 *  Saves 6 bytes per frame vs full Pos_Control — ideal for rapid updates */
void FastPos_Move(uint8_t addr, int32_t clk)
{
    uint8_t cmd[8] = {0};
    uint32_t raw = (uint32_t)clk;
    cmd[0] = addr;
    cmd[1] = 0xFC;
    cmd[2] = (uint8_t)(raw >> 24);
    cmd[3] = (uint8_t)(raw >> 16);
    cmd[4] = (uint8_t)(raw >> 8);
    cmd[5] = (uint8_t)(raw & 0xFF);
    cmd[6] = 0x6B;
    can_SendCmd(cmd, 7);
}

void Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[6] = {0};
    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = (uint8_t)snF;
    cmd[4] = 0x6B;
    can_SendCmd(cmd, 5);
}

void Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[4] = {0};
    cmd[0] = addr;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;
    can_SendCmd(cmd, 4);
}

/* ================================================================
 *  Read system parameters
 * ================================================================ */
void Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[8] = {0};

    cmd[i++] = addr;

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
    case S_PULSES:
        cmd[i++] = 0x32;
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

    cmd[i++] = 0x6B;
    can_SendCmd(cmd, i);
}

/* ================================================================
 *  Homing commands
 * ================================================================ */
void Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0A, 0x6D, 0x6B};
    can_SendCmd(cmd, 4);
}

void Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[5] = {addr, 0x93, 0x88, (uint8_t)svF, 0x6B};
    can_SendCmd(cmd, 5);
}

void Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[5] = {addr, 0x9A, o_mode, (uint8_t)snF, 0x6B};
    can_SendCmd(cmd, 5);
}

void Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x9C, 0x48, 0x6B};
    can_SendCmd(cmd, 4);
}

void Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir,
                          uint16_t o_vel, uint32_t o_tm,
                          uint16_t sl_vel, uint16_t sl_ma,
                          uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    cmd[0] = addr;
    cmd[1] = 0x4C;
    cmd[2] = 0xAE;
    cmd[3] = (uint8_t)svF;
    cmd[4] = o_mode;
    cmd[5] = o_dir;
    cmd[6] = (uint8_t)(o_vel >> 8);
    cmd[7] = (uint8_t)(o_vel & 0xFF);
    cmd[8] = (uint8_t)(o_tm >> 24);
    cmd[9] = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm & 0xFF);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel & 0xFF);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma & 0xFF);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms & 0xFF);
    cmd[18] = (uint8_t)potF;
    cmd[19] = 0x6B;
    can_SendCmd(cmd, 20); // auto-split into 3 CAN frames
}

/* ================================================================
 *  Misc / config
 * ================================================================ */
void Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
    can_SendCmd(cmd, 4);
}

void Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[6] = {addr, 0x46, 0x69, (uint8_t)svF, ctrl_mode, 0x6B};
    can_SendCmd(cmd, 6);
}

void Set_MotorID(uint8_t addr, bool svF, uint8_t new_id)
{
    uint8_t cmd[6] = {addr, 0xAE, 0x4B, (uint8_t)svF, new_id, 0x6B};
    can_SendCmd(cmd, 6);
}

/* ================================================================
 *  Non-blocking wheel velocity dispatch
 * ================================================================ */
typedef struct
{
    int16_t vfl;
    int16_t vfr;
    int16_t vrl;
    int16_t vrr;
    uint8_t dirfl;
    uint8_t dirfr;
    uint8_t dirrl;
    uint8_t dirrr;
    uint8_t step;
    uint32_t next_due_ms;
    bool active;
    bool use_dirs;
} WheelVelocityQueue_t;

static volatile WheelVelocityQueue_t wheelQueue = {0};
static const uint32_t wheelVelocityGapMs = 1U;

/* Default motor directions (permanent, hardware orientation)
 * Index: addr-1. For omniwheels we preset which way 'positive' maps.
 * TOP_LEFT  = CW, TOP_RIGHT = CW, BACK_LEFT = CCW, BACK_RIGHT = CCW
 */
static const uint8_t motorDefaultDir[MAX_MOTORS] = {
    MOTOR_DIR_CCW, /* addr 0x01 TOP_LEFT_MOTOR */
    MOTOR_DIR_CW,  /* addr 0x02 TOP_RIGHT_MOTOR */
    MOTOR_DIR_CCW, /* addr 0x03 BACK_LEFT_MOTOR */
    MOTOR_DIR_CW,  /* addr 0x04 BACK_RIGHT_MOTOR */
    MOTOR_DIR_CW,  /* addr 0x05 VER_MOTOR (default) */
    MOTOR_DIR_CW   /* addr 0x06 HOR_MOTOR (default) */
};

static void SendSingleWheelVelocity(uint8_t motorAddr, int16_t targetVelocity)
{
    /* Use the pre-set default direction for this motor and flip when velocity is negative */
    uint8_t idx = (motorAddr > 0) ? (motorAddr - 1) : 0;
    uint8_t base_dir = MOTOR_DIR_CW;
    if (idx < MAX_MOTORS)
        base_dir = motorDefaultDir[idx];

    uint8_t dir = (targetVelocity < 0) ? (base_dir ^ 1) : base_dir;
    uint16_t mag = (uint16_t)ABS(targetVelocity);
    Vel_Control(motorAddr, dir, mag, 0, false);
}

static void SendSingleWheelVelocity_Dir(uint8_t motorAddr, int16_t targetVelocity, uint8_t dir)
{
    Vel_Control(motorAddr, dir ? MOTOR_DIR_CCW : MOTOR_DIR_CW, (uint16_t)ABS(targetVelocity), 0, false);
}

void MotorVelocity_Task(void)
{
    if (!wheelQueue.active)
        return;

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - wheelQueue.next_due_ms) < 0)
        return;

    switch (wheelQueue.step)
    {
    case 0:
        if (wheelQueue.use_dirs)
            SendSingleWheelVelocity_Dir(TOP_LEFT_MOTOR, wheelQueue.vfl, wheelQueue.dirfl);
        else
            SendSingleWheelVelocity(TOP_LEFT_MOTOR, wheelQueue.vfl);
        break;
    case 1:
        if (wheelQueue.use_dirs)
            SendSingleWheelVelocity_Dir(TOP_RIGHT_MOTOR, wheelQueue.vfr, wheelQueue.dirfr);
        else
            SendSingleWheelVelocity(TOP_RIGHT_MOTOR, wheelQueue.vfr);
        break;
    case 2:
        if (wheelQueue.use_dirs)
            SendSingleWheelVelocity_Dir(BACK_LEFT_MOTOR, wheelQueue.vrl, wheelQueue.dirrl);
        else
            SendSingleWheelVelocity(BACK_LEFT_MOTOR, wheelQueue.vrl);
        break;
    case 3:
        if (wheelQueue.use_dirs)
            SendSingleWheelVelocity_Dir(BACK_RIGHT_MOTOR, wheelQueue.vrr, wheelQueue.dirrr);
        else
            SendSingleWheelVelocity(BACK_RIGHT_MOTOR, wheelQueue.vrr);
        break;
    default:
        wheelQueue.active = false;
        return;
    }

    wheelQueue.step++;
    wheelQueue.next_due_ms = now + wheelVelocityGapMs;

    if (wheelQueue.step >= 4)
        wheelQueue.active = false;
}

void Send_Velocities(int16_t vfl, int16_t vfr, int16_t vrl, int16_t vrr)
{
    wheelQueue.vfl = vfl;
    wheelQueue.vfr = vfr;
    wheelQueue.vrl = vrl;
    wheelQueue.vrr = vrr;
    wheelQueue.use_dirs = false;
    wheelQueue.step = 0;
    wheelQueue.next_due_ms = HAL_GetTick();
    wheelQueue.active = true;
}

void Send_Velocities_WithDirs(int16_t vfl, uint8_t dirfl,
                              int16_t vfr, uint8_t dirfr,
                              int16_t vrl, uint8_t dirrl,
                              int16_t vrr, uint8_t dirrr)
{
    wheelQueue.vfl = vfl;
    wheelQueue.vfr = vfr;
    wheelQueue.vrl = vrl;
    wheelQueue.vrr = vrr;
    wheelQueue.dirfl = dirfl;
    wheelQueue.dirfr = dirfr;
    wheelQueue.dirrl = dirrl;
    wheelQueue.dirrr = dirrr;
    wheelQueue.use_dirs = true;
    wheelQueue.step = 0;
    wheelQueue.next_due_ms = HAL_GetTick();
    wheelQueue.active = true;
}