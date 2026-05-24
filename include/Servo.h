#pragma once
#include "main.h"

#define YAW_SERVO 0
#define CLAW_SERVO 1

#define DEFAULT_TIME 100

// Servo pin macros
#define SERVO1_HIGH() HAL_GPIO_WritePin(SERVO_1_GPIO_Port,SERVO_1_Pin,GPIO_PIN_SET)
#define SERVO1_LOW() HAL_GPIO_WritePin(SERVO_1_GPIO_Port,SERVO_1_Pin,GPIO_PIN_RESET)

#define SERVO2_HIGH() HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_SET)
#define SERVO2_LOW()  HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_RESET)

// Function declarations
void PwmServo_Init(void);
void PwmServo_Set_Angle(uint8_t index, uint8_t angle, uint16_t time);
void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint8_t a3, uint16_t time);
void PwmServo_Handle(void);