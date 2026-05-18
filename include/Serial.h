#pragma once

#include "main.h"
#include <stdint.h>
#include <stdio.h>

/* Set to 1 to use DMA for transmission, 0 for blocking */
#define ENABLE_UART_DMA  1

/* Exposed so the ISR callback in bsp_uart.c can use it */
extern uint8_t RxTemp;
extern UART_HandleTypeDef huart1;

void USART1_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);