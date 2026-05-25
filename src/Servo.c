// Servo.c - fixed

#include "Servo.h"
#include "cmsis_gcc.h"   // for __disable_irq / __enable_irq
#include "Beep.h"

#define SERVO_FRAME_TICKS  2000u
#define SERVO_MIN_TICKS      50u
#define SERVO_MAX_TICKS     250u

static volatile uint16_t servoPulse[2]      = {0, 0};
static volatile uint16_t servoPulse_next[2] = {0, 0};
static volatile uint8_t  pulseUpdated       = 0;

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

// ── Write both shadow regs + flag in one critical section ──────────────────
static inline void latch_next(uint16_t p0, uint16_t p1)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
        servoPulse_next[0] = p0;
        servoPulse_next[1] = p1;
        pulseUpdated        = 1;
    __set_PRIMASK(primask);   // restore, not unconditional enable
}

void PwmServo_Set_Angle(uint8_t index, uint8_t angle, uint16_t time)
{
    if (index >= 2) return;
    // Read-modify: keep the other channel's current next value
    uint16_t p[2] = { servoPulse_next[0], servoPulse_next[1] };
    p[index] = PwmServo_Angle_To_Pulse(angle);
    latch_next(p[0], p[1]);
}

void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint8_t a3, uint16_t time)
{
    latch_next(PwmServo_Angle_To_Pulse(a1),
               PwmServo_Angle_To_Pulse(a2));
}

// Called from TIM7 ISR at 100kHz
void PwmServo_Handle(void)
{
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

    // ── Use >= instead of == to survive any skipped tick ──────────────────
    if (pwmCount >= servoPulse[0]) SERVO1_LOW();
    if (pwmCount >= servoPulse[1]) SERVO2_LOW();

    if (++pwmCount >= SERVO_FRAME_TICKS)
        pwmCount = 0;
}