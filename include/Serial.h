#pragma once

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"

#define TX_RING_BUF_SIZE 512   // power of 2 makes the % free (can use & mask)

/* Set to 1 to use DMA for transmission, 0 for blocking */
#define ENABLE_UART_DMA 0

/* Exposed so the ISR callback in bsp_uart.c can use it */
extern uint8_t RxTemp;
extern UART_HandleTypeDef huart1;

typedef struct {
    uint8_t  buf[TX_RING_BUF_SIZE];
    volatile uint16_t head;   // written by main/task context
    volatile uint16_t tail;   // written by ISR only
} TxRingBuf;

extern TxRingBuf txRing;
extern uint8_t   RxTemp;
extern UART_HandleTypeDef huart1;

void USART1_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);