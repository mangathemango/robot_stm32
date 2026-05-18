#include "Serial.h"
#include "SerialDecoder.h"

/* Single-byte receive buffer used by the interrupt-driven receive */
uint8_t RxTemp = 0;
UART_HandleTypeDef huart1;

/* --------------------------------------------------------------------------
 * USART1_Init
 * Kick off the first interrupt-driven receive so the UART ISR fires when
 * the first byte arrives.
 * -------------------------------------------------------------------------- */
void USART1_Init(void)
{
    // Initialize USART1 - 初始化串口1
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}

/* --------------------------------------------------------------------------
 * USART1_Send_U8
 * Send a single byte over USART1 (blocking with indefinite timeout).
 * -------------------------------------------------------------------------- */
void USART1_Send_U8(uint8_t ch)
{
    // The serial port sends one byte - 串口发送一个字节
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}

/* --------------------------------------------------------------------------
 * USART1_Send_ArrayU8
 * Send a buffer of bytes over USART1.
 * If ENABLE_UART_DMA is 1, uses DMA so the CPU is not blocked.
 * Otherwise falls back to sending byte-by-byte.
 * -------------------------------------------------------------------------- */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    // The serial port sends a string of data - 串口发送一串数据
#if ENABLE_UART_DMA
    HAL_UART_Transmit_DMA(&huart1, BufferPtr, Length);
#else
    while (Length--)
    {
        USART1_Send_U8(*BufferPtr);
        BufferPtr++;
    }
#endif
}

/* --------------------------------------------------------------------------
 * HAL_UART_RxCpltCallback
 * Called automatically by the HAL when a byte has been received.
 * NOTE: In production code you should NOT transmit inside an ISR callback —
 *       it is done here only for quick testing as the PDF warns.
 * -------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE: This function should not be modified; when the callback is needed
     *       the HAL_UART_RxCpltCallback can be implemented in the user file. */

    // Test sending data. In practice, data should NOT be sent during interrupts
    // 测试发送数据，实际应用中不应该在中断中发送数据
    handleSerialData(RxTemp);

    // Continue receiving data - 继续接收数据
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
}

/* --------------------------------------------------------------------------
 * __io_putchar / fputc  — redirect printf to USART1
 * The correct definition is selected automatically based on the toolchain.
 * -------------------------------------------------------------------------- */
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here.
     * e.g. write a character to EVAL_COM1 and loop until end of transmission. */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}