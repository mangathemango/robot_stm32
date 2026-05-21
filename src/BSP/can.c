#include "can.h"
#include "EmmV5.h"
#include "SerialEncoder.h"

/* ---------------------------------------------------------------
 *  Driver state instance
 * --------------------------------------------------------------- */
volatile CAN_t can = {0};
volatile WheelVelSnapshot_t wheelVelSnap = {0};
volatile ArmPosSnapshot_t armPosSnap = {0};

/* ---------------------------------------------------------------
 *  CAN_Start
 * --------------------------------------------------------------- */
HAL_StatusTypeDef CAN_Start(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000; // accept all IDs
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_Start(&hcan) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}

/* ---------------------------------------------------------------
 *  HAL RX callback — parses all supported response types
 *
 *  Frame layout (all responses):
 *    ExtId high byte = motor address
 *    data[0]         = function code
 *    data[1]         = sign byte (0x00 = positive, 0x01 = negative)
 *    data[2..N]      = value bytes, big-endian
 *    data[last]      = 0x6B checksum
 *
 *  Supported function codes:
 *    0x32  input pulse count  (6 bytes: addr + 0x32 + sign + 4 value bytes + 0x6B) — wait, DLC=6
 *    0x35  real-time velocity (5 bytes: addr + 0x35 + sign + 2 value bytes + 0x6B) — DLC=4 in data[]
 *
 *  Note: the CAN frame data[] does NOT include the address byte.
 *  The address comes from ExtId. So:
 *    data[0] = func code
 *    data[1] = sign
 *    data[2..] = value
 * --------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_cb)
{
    HAL_CAN_GetRxMessage(hcan_cb,
                         CAN_RX_FIFO0,
                         (CAN_RxHeaderTypeDef *)&can.rxFrame.header,
                         (uint8_t *)can.rxFrame.data);

    can.rxFrameFlag = true;

    /* Motor address from high byte of extended CAN ID */
    uint8_t addr = (uint8_t)((can.rxFrame.header.ExtId >> 8) & 0xFF);
    if (addr < 1 || addr > MAX_MOTORS)
        return;
    uint8_t idx = addr - 1;

    uint8_t funcCode = can.rxFrame.data[0];
    uint8_t sign = can.rxFrame.data[1];

    if (funcCode == 0x35 && can.rxFrame.header.DLC >= 4)
    {
        uint16_t raw = ((uint16_t)can.rxFrame.data[2] << 8) | (uint16_t)can.rxFrame.data[3];
        can.motor[idx].rpm = (sign == 0x01) ? -(int16_t)raw : (int16_t)raw;
        can.motor[idx].vel_updated = true;

        if (can.motor[TOP_LEFT_MOTOR - 1].vel_updated &&
            can.motor[TOP_RIGHT_MOTOR - 1].vel_updated &&
            can.motor[BACK_LEFT_MOTOR - 1].vel_updated &&
            can.motor[BACK_RIGHT_MOTOR - 1].vel_updated)
        {
            /* Snapshot into a plain struct — no serial call here */
            wheelVelSnap.vfl = can.motor[TOP_LEFT_MOTOR - 1].rpm;
            wheelVelSnap.vfr = can.motor[TOP_RIGHT_MOTOR - 1].rpm;
            wheelVelSnap.vrl = can.motor[BACK_LEFT_MOTOR - 1].rpm;
            wheelVelSnap.vrr = can.motor[BACK_RIGHT_MOTOR - 1].rpm;
            wheelVelSnap.ready = true; /* main loop sees this and sends */

            can.motor[TOP_LEFT_MOTOR - 1].vel_updated = false;
            can.motor[TOP_RIGHT_MOTOR - 1].vel_updated = false;
            can.motor[BACK_LEFT_MOTOR - 1].vel_updated = false;
            can.motor[BACK_RIGHT_MOTOR - 1].vel_updated = false;
        }
    }

    else if (funcCode == 0x32 && can.rxFrame.header.DLC >= 6)
    {
        uint32_t raw = ((uint32_t)can.rxFrame.data[2] << 24) |
                       ((uint32_t)can.rxFrame.data[3] << 16) |
                       ((uint32_t)can.rxFrame.data[4] << 8) |
                       (uint32_t)can.rxFrame.data[5];

        can.motor[idx].pulses = (sign == 0x01) ? -(int32_t)raw : (int32_t)raw;
        can.motor[idx].pos_updated = true;

        float revs = (float)can.motor[idx].pulses / (float)PULSES_PER_REV;
        int32_t scaled = (int32_t)(revs * 1000.0f);
        if (scaled < 0)
            scaled = -scaled;
        if (scaled > 10000)
            scaled = 10000;

        if (addr == VER_MOTOR)
        {
            armPosSnap.ver_pulses = can.motor[idx].pulses; // store raw int32
            armPosSnap.ver_ready = true;
            can.motor[idx].pos_updated = false;
        }
        else if (addr == HOR_MOTOR)
        {
            armPosSnap.hor_pulses = can.motor[idx].pulses;
            armPosSnap.hor_ready = true;
            can.motor[idx].pos_updated = false;
        }
    }
}

bool CAN_TakeWheelVelocitySnapshot(int16_t *vfl, int16_t *vfr, int16_t *vrl, int16_t *vrr)
{
    if (!wheelVelSnap.ready)
        return false;

    __disable_irq();
    if (!wheelVelSnap.ready)
    {
        __enable_irq();
        return false;
    }

    *vfl = wheelVelSnap.vfl;
    *vfr = wheelVelSnap.vfr;
    *vrl = wheelVelSnap.vrl;
    *vrr = wheelVelSnap.vrr;
    wheelVelSnap.ready = false;
    __enable_irq();
    return true;
}

/* ---------------------------------------------------------------
 *  can_SendCmd
 * --------------------------------------------------------------- */
void can_SendCmd(volatile uint8_t *cmd, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8];
    uint32_t txMailbox;

    uint8_t payloadSent = 0;
    uint8_t payloadTotal = len - 2;
    uint8_t packetNum = 0;

    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;

    while (payloadSent < payloadTotal)
    {
        uint8_t remaining = payloadTotal - payloadSent;

        txHeader.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packetNum;
        txData[0] = cmd[1]; // function code always first byte of frame

        uint8_t chunkSize;

        if (remaining <= 7)
        {
            chunkSize = remaining;
            for (uint8_t i = 0; i < chunkSize; i++)
                txData[i + 1] = cmd[payloadSent + 2 + i];
            txHeader.DLC = chunkSize + 1;
        }
        else
        {
            chunkSize = 7;
            for (uint8_t i = 0; i < chunkSize; i++)
                txData[i + 1] = cmd[payloadSent + 2 + i];
            txHeader.DLC = 8;
        }

        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        {
        }
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

        payloadSent += chunkSize;
        packetNum++;
    }
}

/* ---------------------------------------------------------------
 *  CAN_ReadVelocity — blocking, returns signed RPM
 *  Returns 0x7FFF on timeout
 * --------------------------------------------------------------- */
int16_t CAN_ReadVelocity(uint8_t addr, uint32_t timeout_ms)
{
    if (addr < 1 || addr > MAX_MOTORS)
        return 0x7FFF;
    uint8_t idx = addr - 1;

    can.motor[idx].vel_updated = false;
    Read_Sys_Params(addr, S_VEL); // sends function code 0x35

    uint32_t start = HAL_GetTick();
    while (!can.motor[idx].vel_updated)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return 0x7FFF;
    }
    return can.motor[idx].rpm;
}

/* ---------------------------------------------------------------
 *  CAN_ReadPulses — blocking, returns signed pulse count
 *  Same unit as the 'clk' param in Emm_V5_Pos_Control()
 *  Returns INT32_MIN on timeout
 * --------------------------------------------------------------- */
float CAN_ReadRevs(uint8_t addr, uint32_t timeout_ms)
{
    if (addr < 1 || addr > MAX_MOTORS)
        return -99999.0f;
    uint8_t idx = addr - 1;

    can.motor[idx].pos_updated = false;
    Read_Sys_Params(addr, S_PULSES);

    uint32_t start = HAL_GetTick();
    while (!can.motor[idx].pos_updated)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return -99999.0f;
    }

    return (float)can.motor[idx].pulses / PULSES_PER_REV;
}