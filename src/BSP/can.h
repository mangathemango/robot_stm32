#ifndef __CAN_H
#define __CAN_H

#include "main.h"
#include <stdbool.h>
#include <string.h>

/* ---------------------------------------------------------------
 *  CAN RX Frame struct (mirrors HAL's CAN_RxHeaderTypeDef + data)
 * --------------------------------------------------------------- */
typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_RxFrame_t;

/* ---------------------------------------------------------------
 *  CAN driver state
 * --------------------------------------------------------------- */
typedef struct
{
    CAN_RxFrame_t rxFrame;
    volatile bool rxFrameFlag; // set true by RX interrupt, cleared by user
} CAN_t;

/* ---------------------------------------------------------------
 *  Extern handle — defined in main.c by CubeMX
 * --------------------------------------------------------------- */
extern CAN_HandleTypeDef hcan;

/* ---------------------------------------------------------------
 *  Extern driver state — defined in can.c
 * --------------------------------------------------------------- */
extern volatile CAN_t can;

/* ---------------------------------------------------------------
 *  Public API
 * --------------------------------------------------------------- */

/**
 * @brief  Call once after MX_CAN_Init() to configure the RX filter
 *         and start the peripheral + interrupts.
 */
HAL_StatusTypeDef CAN_Start(void);

/**
 * @brief  Send a raw command buffer following the ZDT packet protocol.
 *         cmd[0]  = motor address (Addr)
 *         cmd[1]  = function code
 *         cmd[2…] = payload + checksum byte at the end
 *         len     = total number of bytes in cmd[]
 */
void can_SendCmd(volatile uint8_t *cmd, uint8_t len);

#endif /* __CAN_H */