#ifndef __CAN_H
#define __CAN_H

#include "main.h"
#include <stdbool.h>
#include <string.h>

/* ---------------------------------------------------------------
 *  CAN RX Frame
 * --------------------------------------------------------------- */
typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_RxFrame_t;

/* ---------------------------------------------------------------
 *  Per-motor data — populated automatically by the RX callback
 * --------------------------------------------------------------- */
#define MAX_MOTORS 6 // supports addresses 0x01 to 0x06

typedef struct
{
    int16_t rpm;      // signed RPM  (from 0x35 response)
    bool vel_updated; // true when fresh velocity data arrived

    int32_t pulses;   // signed pulse count (from 0x32 response)
    bool pos_updated; // true when fresh pulse data arrived
} MotorData_t;

/* ---------------------------------------------------------------
 *  CAN driver state
 * --------------------------------------------------------------- */
typedef struct
{
    CAN_RxFrame_t rxFrame;
    volatile bool rxFrameFlag;
    MotorData_t motor[MAX_MOTORS]; // motor[0] = addr 0x01, etc.
} CAN_t;

/* ---------------------------------------------------------------
 *  Externs
 * --------------------------------------------------------------- */
extern CAN_HandleTypeDef hcan;
extern volatile CAN_t can;

/* ---------------------------------------------------------------
 *  Public API
 * --------------------------------------------------------------- */
HAL_StatusTypeDef CAN_Start(void);
void can_SendCmd(volatile uint8_t *cmd, uint8_t len);
void CAN_ProcessDeferredReports(void);

/**
 * @brief  Request and return real-time velocity (RPM) from one motor.
 *         Returns 0x7FFF on timeout.
 */
int16_t CAN_ReadVelocity(uint8_t addr, uint32_t timeout_ms);

/**
 * @brief  Request and return the accumulated input pulse count from one motor.
 *         This matches the 'clk' unit used in Emm_V5_Pos_Control().
 *         Returns INT32_MIN on timeout.
 */
float CAN_ReadRevs(uint8_t addr, uint32_t timeout_ms);

#endif /* __CAN_H */