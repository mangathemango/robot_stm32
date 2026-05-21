#include "SerialDecoder.h"
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "Servo.h"
#include "can.h"
#include "EmmV5.h"
uint8_t buffer[300];
static int bufIndex = 0; // renamed from 'index' — index() is a POSIX stdlib name

// ── handleSerialData ──────────────────────────────────────────────────────────
// Called for every incoming byte.
// Builds up buffer[] and fires handlePacket() once a complete packet arrives.
//
// Buffer layout while filling:
//   [0]        START byte  (0x67)
//   [1]        ID
//   [2]        LEN         (number of DATA bytes that follow)
//   [3..3+N-1] DATA        (N = LEN bytes)
//   [3+N]      CHECKSUM
// ─────────────────────────────────────────────────────────────────────────────
void handleSerialData(uint8_t byte)
{
    // ── Sync: always reset on a fresh start byte ──────────────────────────────
    if (byte == START_BYTE)
    {
        bufIndex = 0;
    }

    // ── Guard: drop bytes that arrive before we have seen a start byte ────────
    if (bufIndex == 0 && byte != START_BYTE)
    {
        return;
    }

    // ── Overflow guard ────────────────────────────────────────────────────────
    if (bufIndex >= (int)sizeof(buffer))
    {
        bufIndex = 0; // reset and wait for next start byte
        return;
    }

    buffer[bufIndex++] = byte;

    // ── Wait until we at least have START + ID + LEN ─────────────────────────
    if (bufIndex < 3)
    {
        return;
    }

    uint8_t len = buffer[PKT_LEN];          // number of DATA bytes
    int expectedTotal = PKT_DATA + len + 1; // header(3) + data(len) + checksum(1)

    // ── Wait until the full packet (including checksum) has arrived ───────────
    if (bufIndex < expectedTotal)
    {
        return;
    }

    // ── Checksum validation ───────────────────────────────────────────────────
    // XOR every byte from START up to and including the last DATA byte.
    // The result must match the checksum byte that follows.
    uint8_t computed = 0;
    for (int i = PKT_START; i < PKT_DATA + len; i++)
    {
        computed ^= buffer[i];
    }

    uint8_t received = buffer[PKT_DATA + len];

    if (computed != received)
    {
        // Bad checksum — discard packet and re-sync
        printf("[SerialDecoder] Checksum error: computed 0x%02X, received 0x%02X\n",
               computed, received);
        bufIndex = 0;
        return;
    }

    // ── All checks passed — hand off to packet handler ────────────────────────
    handlePacket();
    bufIndex = 0; // ready for next packet
}

// ── handlePacket ──────────────────────────────────────────────────────────────
// At this point buffer[] holds a fully validated packet:
//   buffer[PKT_ID]      command ID
//   buffer[PKT_LEN]     data length
//   buffer[PKT_DATA ..] data bytes (little-endian, unsigned unless range includes negatives)
// ─────────────────────────────────────────────────────────────────────────────
void handlePacket(void)
{
    uint8_t id = buffer[PKT_ID];
    uint8_t len = buffer[PKT_LEN];
    uint8_t *data = &buffer[PKT_DATA];

    switch (id)
    {
    // ── 0x01  Set_Yaw_Servo_Angle ─────────────────────────────────────────
    // LEN = 0x01  |  data[0] = Angle (0-180, uint8)
    case CMD_SET_YAW_SERVO_ANGLE:
    {
        if (len != 0x01)
            break;
        uint8_t angle = data[0]; // 0–180
        // TODO: Set_Yaw_Servo_Angle(angle);
        PwmServo_Set_Angle(YAW_SERVO, angle);
        printf("[PKT] Set_Yaw_Servo_Angle: %u\n", angle);
        break;
    }

    // ── 0x02  Set_Claw_Servo_Angle ────────────────────────────────────────
    // LEN = 0x01  |  data[0] = Angle (0-180, uint8)
    case CMD_SET_CLAW_SERVO_ANGLE:
    {
        if (len != 0x01)
            break;
        uint8_t angle = data[0]; // 0–180
        // TODO: Set_Claw_Servo_Angle(angle);
        PwmServo_Set_Angle(CLAW_SERVO, angle);
        printf("[PKT] Set_Claw_Servo_Angle: %u\n", angle);
        break;
    }

    // ── 0x03  Set_Display_Text ────────────────────────────────────────────
    // LEN = variable  |  data = ASCII string (NOT null-terminated in packet)
    case CMD_SET_DISPLAY_TEXT:
    {
        // Safely copy into a local null-terminated buffer for string ops
        char text[256] = {0};
        uint8_t copyLen = (len < sizeof(text) - 1) ? len : (uint8_t)(sizeof(text) - 1);
        memcpy(text, data, copyLen);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 15);
        ssd1306_WriteString(text, Font_16x26, White);
        ssd1306_UpdateScreen();
        printf("[PKT] Set_Display_Text: \"%s\"\n", text);
        break;
    }

    // ── 0x06  Set_Vertical_Arm_Position ──────────────────────────────────
    // LEN = 0x02  |  data[0..1] = Position (0-10000, uint16 little-endian)
    case CMD_SET_VERTICAL_ARM_POSITION:
    {
        if (len != 0x02)
            break;
        uint16_t position = (uint16_t)(data[0] | (data[1] << 8)); // little-endian

        Pos_Control(VER_MOTOR, MOTOR_DIR_CCW, 1000, 50, position, true, false);

        printf("[PKT] Set_Vertical_Arm_Position: %u\n", position);
        break;
    }

    // ── 0x07  Set_Horizontal_Arm_Position ────────────────────────────────
    // LEN = 0x02  |  data[0..1] = Position (0-10000, uint16 little-endian)
    case CMD_SET_HORIZONTAL_ARM_POSITION:
    {
        if (len != 0x02)
            break;
        uint16_t position = (uint16_t)(data[0] | (data[1] << 8)); // little-endian

        Pos_Control(HOR_MOTOR, MOTOR_DIR_CCW, 1000, 50, position, true, false);

        printf("[PKT] Set_Horizontal_Arm_Position: %u\n", position);
        break;
    }

    // ── 0x08  Beep ────────────────────────────────────────────────────────
    // LEN = 0x00  |  no data bytes
    case CMD_BEEP:
    {
        if (len != 0x00)
            break;
        Beep_Start(10);
        printf("[PKT] Beep\n");
        break;
    }

    // ── 0x09  Set_Wheel_Target_Velocities ────────────────────────────────
    // LEN = 0x08  |  4 x int16 little-endian: vfl, vfr, vrl, vrr (-10000 to 10000)
    case CMD_SET_WHEEL_TARGET_VELOCITIES:
    {
        if (len != 0x08)
            break;
        // Range includes negatives → signed int16
        int16_t vfl = (int16_t)(data[0] | (data[1] << 8));
        int16_t vfr = (int16_t)(data[2] | (data[3] << 8));
        int16_t vrl = (int16_t)(data[4] | (data[5] << 8));
        int16_t vrr = (int16_t)(data[6] | (data[7] << 8));

        Send_Velocities(vfl, vfr, vrl, vrr);

        printf("[PKT] Set_Wheel_Target_Velocities: vfl=%d vfr=%d vrl=%d vrr=%d\n",
               vfl, vfr, vrl, vrr);
        break;
    }

    default:
        printf("[SerialDecoder] Unknown command ID: 0x%02X\n", id);
        break;
    }
}