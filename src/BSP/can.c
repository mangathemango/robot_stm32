#include "can.h"

/* ---------------------------------------------------------------
 *  Driver state instance
 * --------------------------------------------------------------- */
volatile CAN_t can = {0};

/* ---------------------------------------------------------------
 *  CAN_Start
 *  Call this once in main(), right after MX_CAN_Init().
 *
 *  It configures a "pass everything" filter (we only have one motor
 *  type on the bus so no need to hardware-filter by ID), then starts
 *  the peripheral and activates the FIFO0 RX interrupt.
 * --------------------------------------------------------------- */
HAL_StatusTypeDef CAN_Start(void)
{
    CAN_FilterTypeDef filter = {0};

    /* Pass-all filter on FIFO 0 */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000; // mask = 0 → accept any ID
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Enable RX FIFO0 message-pending interrupt */
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ---------------------------------------------------------------
 *  HAL RX callback — called from USB_LP_CAN1_RX0_IRQHandler
 *  (already wired in stm32f1xx_it.c by CubeMX)
 * --------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_cb)
{
    HAL_CAN_GetRxMessage(hcan_cb,
                         CAN_RX_FIFO0,
                         (CAN_RxHeaderTypeDef *)&can.rxFrame.header,
                         (uint8_t *)can.rxFrame.data);

    can.rxFrameFlag = true;
}

/* ---------------------------------------------------------------
 *  can_SendCmd
 *
 *  Protocol recap:
 *    cmd[0]            = Addr  (motor address, e.g. 0x01)
 *    cmd[1]            = function code  (first data byte of every packet)
 *    cmd[2 … len-1]    = payload + final 0x6B checksum byte
 *
 *  Extended CAN ID = (Addr << 8) | packetNumber
 *
 *  Each CAN frame carries up to 8 bytes:
 *    byte 0 of frame  = function code  (cmd[1], repeated every packet)
 *    bytes 1-7        = up to 7 payload bytes
 *
 *  For commands whose payload fits in one packet (≤ 7 payload bytes
 *  after the function code) a single frame is sent.
 *  Longer commands are split; packetNumber increments per frame.
 * --------------------------------------------------------------- */
void can_SendCmd(volatile uint8_t *cmd, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8];
    uint32_t txMailbox;

    uint8_t payloadSent = 0;        // how many bytes of cmd[2..] already sent
    uint8_t payloadTotal = len - 2; // total payload bytes (excludes addr + func)
    uint8_t packetNum = 0;

    txHeader.IDE = CAN_ID_EXT;
    txHeader.RTR = CAN_RTR_DATA;

    while (payloadSent < payloadTotal)
    {
        uint8_t remaining = payloadTotal - payloadSent;

        /* Build extended ID */
        txHeader.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packetNum;

        /* First byte of every frame is always the function code */
        txData[0] = cmd[1];

        uint8_t chunkSize; // payload bytes going into this frame (max 7)

        if (remaining <= 7)
        {
            /* Last (or only) packet — fits in one frame */
            chunkSize = remaining;
            for (uint8_t i = 0; i < chunkSize; i++)
            {
                txData[i + 1] = cmd[payloadSent + 2 + i];
            }
            txHeader.DLC = chunkSize + 1; // +1 for function code byte
        }
        else
        {
            /* More than 7 bytes left — fill a full 8-byte frame */
            chunkSize = 7;
            for (uint8_t i = 0; i < chunkSize; i++)
            {
                txData[i + 1] = cmd[payloadSent + 2 + i];
            }
            txHeader.DLC = 8;
        }

        /* Wait until a TX mailbox is free, then send */
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        {
        }

        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);

        payloadSent += chunkSize;
        packetNum++;
    }
}