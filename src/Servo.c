#include "Servo.h"

static uint16_t servoPulse[3];
static uint16_t pwmCount = 0;
static int16_t servoTimer[3]; //in ms
static uint8_t msTickCount = 0;

// Convert angle (0-180) to pulse width

// 0 deg  -> 0.5ms
// 180 deg -> 2.5ms
static uint16_t PwmServo_Angle_To_Pulse(uint8_t angle)
{
    return 50 + (angle * 200 / 180);
}

void PwmServo_Init(void)
{
    // Start all servos at 90 degrees
    PwmServo_Set_Angle_All(0, 0, 0, 100);
}

void PwmServo_Set_Angle(uint8_t index, uint8_t angle, uint16_t time)
{
    if(index > 1)
        return;

    if(angle > 180)
        angle = 180;

    servoPulse[index] = PwmServo_Angle_To_Pulse(angle);
    // servoTimer[index] = time;
}

void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint8_t a3, uint16_t time)
{
    PwmServo_Set_Angle(0, a1, time);
    PwmServo_Set_Angle(1, a2, time);
}

void PwmServo_Handle(void)
{


    pwmCount++;

    if(pwmCount >= 2000)
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