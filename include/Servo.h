#pragma once
#include "main.h"

// Servo pin macros
#define SERVO1_HIGH() HAL_GPIO_WritePin(SERVO_1_GPIO_Port,SERVO_1_Pin,GPIO_PIN_SET)
#define SERVO1_LOW() HAL_GPIO_WritePin(SERVO_1_GPIO_Port,SERVO_1_Pin,GPIO_PIN_RESET)

#define SERVO2_HIGH() HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_SET)
#define SERVO2_LOW()  HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_RESET)

#define SERVO3_HIGH() HAL_GPIO_WritePin(SERVO_3_GPIO_Port, SERVO_3_Pin, GPIO_PIN_SET)
#define SERVO3_LOW()  HAL_GPIO_WritePin(SERVO_3_GPIO_Port, SERVO_3_Pin, GPIO_PIN_RESET)

#define SERVO4_HIGH() HAL_GPIO_WritePin(SERVO_4_GPIO_Port, SERVO_4_Pin, GPIO_PIN_SET)
#define SERVO4_LOW()  HAL_GPIO_WritePin(SERVO_4_GPIO_Port, SERVO_4_Pin, GPIO_PIN_RESET)

// Function declarations
void PwmServo_Init(void);
void PwmServo_Set_Angle(uint8_t index, uint8_t angle);
void PwmServo_Set_Angle_All(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4);
void PwmServo_Handle(void);