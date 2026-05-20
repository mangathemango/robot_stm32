#include "Servo.h"

static uint16_t servoPulse[4];
static uint16_t pwmCount = 0;

// Convert angle (0-180) to pulse width

// 0 deg  -> 0.5ms
// 180 deg -> 2.5ms
static uint16_t PwmServo_Angle_To_Pulse(uint8_t angle)
{
    return 5 + (angle * 20 / 180);
}

void PwmServo_Init(void)
{
    // Start all servos at 90 degrees
    PwmServo_Set_Angle_All(90, 90, 90, 90);
}

void PwmServo_Set_Angle(uint8_t index, uint8_t angle)
{
    if(index > 3)
        return;

    if(angle > 180)
        angle = 180;

    servoPulse[index] = PwmServo_Angle_To_Pulse(angle);
}

void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
    PwmServo_Set_Angle(0, a1);
    PwmServo_Set_Angle(1, a2);
    PwmServo_Set_Angle(2, a3);
    PwmServo_Set_Angle(3, a4);
}

void PwmServo_Handle(void)
{
    pwmCount++;

    if(pwmCount >= 200)
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

    // Servo 3
    if(pwmCount < servoPulse[2])
        SERVO3_HIGH();
    else
        SERVO3_LOW();

    // Servo 4
    if(pwmCount < servoPulse[3])
        SERVO4_HIGH();
    else
        SERVO4_LOW();
}