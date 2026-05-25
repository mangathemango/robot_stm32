#include "Servo.h"

#define SERVO_FRAME_TICKS     2000u
#define SERVO_MIN_PULSE_TICKS   50u
#define SERVO_MAX_PULSE_TICKS  250u
#define SERVO_DEFAULT_ANGLE     90u

static uint16_t servoPulse[2];
static uint16_t pwmCount = 0;

// Convert angle (0-180) to pulse width in TIM7 ticks.
// TIM7 ticks are 10us, so 50 ticks = 0.5ms and 250 ticks = 2.5ms.
static uint16_t PwmServo_Angle_To_Pulse(uint8_t angle)
{
    if (angle > 180)
        angle = 180;

    return SERVO_MIN_PULSE_TICKS +
           (uint16_t)((uint32_t)angle * (SERVO_MAX_PULSE_TICKS - SERVO_MIN_PULSE_TICKS) / 180u);
}

void PwmServo_Init(void)
{
    // Start both servos in the neutral position.
    PwmServo_Set_Angle_All(SERVO_DEFAULT_ANGLE, SERVO_DEFAULT_ANGLE, DEFAULT_TIME);
}

void PwmServo_Set_Angle(uint8_t index, uint8_t angle, uint16_t time)
{
    if (index >= 2)
        return;

    servoPulse[index] = PwmServo_Angle_To_Pulse(angle);
}

void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint16_t time)
{
    PwmServo_Set_Angle(0, a1, time);
    PwmServo_Set_Angle(1, a2, time);
}

void PwmServo_Handle(void)
{


    pwmCount++;

    if (pwmCount >= SERVO_FRAME_TICKS)
        pwmCount = 0;

    // Servo 1
    if(pwmCount < servoPulse[0])
        SERVO1_HIGH();
    else
        SERVO1_LOW();

    // Servo 2
    if(pwmCount < servoPulse[1])
        SERVO2_HIGH();
    else
        SERVO2_LOW();
}