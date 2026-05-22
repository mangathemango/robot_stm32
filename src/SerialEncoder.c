#include "SerialEncoder.h"
#include "Serial.h"
#include <string.h>

// ── Internal helper ───────────────────────────────────────────────────────────
// Builds and sends a complete packet:
//   [START | ID | LEN | data[0..len-1] | CHECKSUM]
// Checksum = XOR of every byte from START through end of data.
// ─────────────────────────────────────────────────────────────────────────────
static void send_packet(uint8_t id, const uint8_t *data, uint8_t len)
{
    // Maximum packet size: 1(start) + 1(id) + 1(len) + 255(data) + 1(checksum)
    uint8_t packet[259];
    uint8_t pos = 0;

    packet[pos++] = START_BYTE;
    packet[pos++] = id;
    packet[pos++] = len;

    for (uint8_t i = 0; i < len; i++)
    {
        packet[pos++] = data[i];
    }

    // Compute checksum: XOR of START ^ ID ^ LEN ^ data bytes
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < pos; i++)
    {
        checksum ^= packet[i];
    }
    packet[pos++] = checksum;

    USART1_Send_ArrayU8(packet, pos);
}

// ── 0x50  Log ─────────────────────────────────────────────────────────────────
// LEN = variable  |  data = ASCII string (no null terminator in packet)
void Serial_Send_Log(const char *text)
{
    uint8_t len = (uint8_t)strlen(text); // truncates silently if >255 — fine for logs
    send_packet(CMD_OUT_LOG, (const uint8_t *)text, len);
}

// ── 0x51  WheelVelocities ─────────────────────────────────────────────────────
// LEN = 0x08  |  4 x int16 little-endian: vfl, vfr, vrl, vrr (-10000 to 10000)
void Serial_Send_WheelVelocities(int16_t vfl, int16_t vfr, int16_t vrl, int16_t vrr)
{
    uint8_t data[8];
    vfl = -vfl;
    vrl = -vrl;
    // Little-endian packing
    data[0] = (uint8_t)(vfl & 0xFF);
    data[1] = (uint8_t)((vfl >> 8) & 0xFF);
    data[2] = (uint8_t)(vfr & 0xFF);
    data[3] = (uint8_t)((vfr >> 8) & 0xFF);
    data[4] = (uint8_t)(vrl & 0xFF);
    data[5] = (uint8_t)((vrl >> 8) & 0xFF);
    data[6] = (uint8_t)(vrr & 0xFF);
    data[7] = (uint8_t)((vrr >> 8) & 0xFF);
    send_packet(CMD_OUT_WHEEL_VELOCITIES, data, 8);
}

// ── 0x52  Key1 ────────────────────────────────────────────────────────────────
// LEN = 0x00  |  no data bytes
void Serial_Send_Key1(void)
{
    send_packet(CMD_OUT_KEY1, NULL, 0);
}

// ── 0x53  HorizontalArmPosition ───────────────────────────────────────────────
// LEN = 0x02  |  uint16 little-endian (0-10000)
void Serial_Send_HorizontalArmPosition(uint16_t position)
{
    uint8_t data[2];
    data[0] = (uint8_t)(position & 0xFF);
    data[1] = (uint8_t)((position >> 8) & 0xFF);
    send_packet(CMD_OUT_HORIZONTAL_ARM_POSITION, data, 2);
}

// ── 0x54  VerticalArmPosition ─────────────────────────────────────────────────
// LEN = 0x02  |  uint16 little-endian (0-10000)
void Serial_Send_VerticalArmPosition(uint16_t position)
{
    uint8_t data[2];
    data[0] = (uint8_t)(position & 0xFF);
    data[1] = (uint8_t)((position >> 8) & 0xFF);
    send_packet(CMD_OUT_VERTICAL_ARM_POSITION, data, 2);
}