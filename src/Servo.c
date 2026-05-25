// Servo.c
#include "Servo.h"

// TIM7: PSC=71 ARR=9 → 10us tick, 100kHz ISR
// Frame = 2000 ticks = 20ms
// Pulse range: 50 ticks (0.5ms) to 230 ticks (2.3ms) = 180 steps = 1 deg/step
#define SERVO_FRAME_TICKS    2000u
#define SERVO_MIN_TICKS        50u   // 0.5ms
#define SERVO_MAX_TICKS       250u   // 2.3ms  (180 steps × 1 tick = 180 ticks range)
#define SERVO_DEFAULT_ANGLE    90u

// Shadow registers — written from main, swapped in at frame boundary (atomic swap)
static volatile uint16_t servoPulse[2]       = {0, 0};
static volatile uint16_t servoPulse_next[2]  = {0, 0};
static volatile uint8_t  pulseUpdated        = 0;

static uint16_t pwmCount = 0;

static uint16_t PwmServo_Angle_To_Pulse(uint8_t angle)
{
    if (angle > 180) angle = 180;
    return SERVO_MIN_TICKS +
           (uint16_t)((uint32_t)angle * (SERVO_MAX_TICKS - SERVO_MIN_TICKS) / 180u);
}

void PwmServo_Init(void)
{
    servoPulse[0] = servoPulse_next[0] = PwmServo_Angle_To_Pulse(180);
    servoPulse[1] = servoPulse_next[1] = PwmServo_Angle_To_Pulse(160);
}

// Called from main — safe, uses shadow register
void PwmServo_Set_Angle(uint8_t index, uint8_t angle, uint16_t time)
{
    if (index >= 2) return;
    servoPulse_next[index] = PwmServo_Angle_To_Pulse(angle);
    pulseUpdated = 1;
}

void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint16_t time)
{
    servoPulse_next[0] = PwmServo_Angle_To_Pulse(a1);
    servoPulse_next[1] = PwmServo_Angle_To_Pulse(a2);
    pulseUpdated = 1;
}

// Called from TIM7 ISR at 100kHz
void PwmServo_Handle(void)
{
    // At frame boundary: go HIGH and latch new pulse widths atomically
    if (pwmCount == 0)
    {
        if (pulseUpdated)
        {
            servoPulse[0] = servoPulse_next[0];
            servoPulse[1] = servoPulse_next[1];
            pulseUpdated  = 0;
        }
        SERVO1_HIGH();
        SERVO2_HIGH();
    }

    // Pull each servo LOW exactly when its pulse expires
    if (pwmCount == servoPulse[0]) SERVO1_LOW();
    if (pwmCount == servoPulse[1]) SERVO2_LOW();

    if (++pwmCount >= SERVO_FRAME_TICKS)
        pwmCount = 0;
}