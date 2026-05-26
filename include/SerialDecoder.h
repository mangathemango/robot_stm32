#pragma once

#include <stdint.h>
#include <stdio.h>
#include "Beep.h"

// Future includes when implementations exist:
// #include "Motor.h"
// #include "Servo.h"
// #include "Display.h"

// ── Protocol constants ────────────────────────────────────────────────────────
#define START_BYTE 0x67

// Packet field offsets inside buffer[]
#define PKT_START 0
#define PKT_ID 1
#define PKT_LEN 2
#define PKT_DATA 3 // data starts at byte 3
#define RING_BUF_SIZE 256

// ── Command IDs ───────────────────────────────────────────────────────────────
typedef enum
{
    CMD_SET_YAW_SERVO_ANGLE = 0x01,
    CMD_SET_CLAW_SERVO_ANGLE = 0x02,
    CMD_SET_DISPLAY_TEXT = 0x03,
    CMD_SET_VERTICAL_ARM_POSITION = 0x06,
    CMD_SET_HORIZONTAL_ARM_POSITION = 0x07,
    CMD_BEEP = 0x08,
    CMD_SET_WHEEL_TARGET_VELOCITIES = 0x09,
} CommandID;

// ── Shared buffer ─────────────────────────────────────────────────────────────
// Layout: [START | ID | LEN | DATA... | CHECKSUM]
extern uint8_t buffer[300];

// ── Public API ────────────────────────────────────────────────────────────────
void handleSerialData(uint8_t byte);
void handlePacket(void);

// SerialDecoder.h additions
typedef struct {
    uint8_t  buf[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuf;

extern RingBuf uartRing;
extern volatile uint8_t packetReady;
extern uint8_t packetBuf[300];
extern uint8_t packetLen;

