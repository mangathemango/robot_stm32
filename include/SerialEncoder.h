#pragma once

#include <stdint.h>
#include <stdio.h>

// ── Protocol constants ────────────────────────────────────────────────────────
// START_BYTE already defined in SerialDecoder.h — only define if standalone
#ifndef START_BYTE
    #define START_BYTE 0x67
#endif

// ── Outgoing Command IDs (STM32 -> Pi) ───────────────────────────────────────
typedef enum {
    CMD_OUT_LOG                      = 0x50,
    CMD_OUT_WHEEL_VELOCITIES         = 0x51,
    CMD_OUT_KEY1                     = 0x52,
    CMD_OUT_HORIZONTAL_ARM_POSITION  = 0x53,
    CMD_OUT_VERTICAL_ARM_POSITION    = 0x54,
} OutCommandID;

// ── Public API ────────────────────────────────────────────────────────────────

// Send a variable-length ASCII log string
void Serial_Send_Log(const char *text);

// Send 4 wheel velocities (-10000 to 10000 each, int16 little-endian)
void Serial_Send_WheelVelocities(int16_t vfl, int16_t vfr, int16_t vrl, int16_t vrr);

// Send Key1 event (no data bytes)
void Serial_Send_Key1(void);

// Send horizontal arm position (0-10000, uint16 little-endian)
void Serial_Send_HorizontalArmPosition(uint16_t position);

// Send vertical arm position (0-10000, uint16 little-endian)
void Serial_Send_VerticalArmPosition(uint16_t position);