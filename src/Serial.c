#include "Serial.h"
#include "SerialDecoder.h"

/* Single-byte receive buffer used by the interrupt-driven receive */
uint8_t RxTemp = 0;
UART_HandleTypeDef huart1;
TxRingBuf txRing = {0};

/* --------------------------------------------------------------------------
 * tx_push  — copy one byte into the TX ring buffer.
 * Called from any context (main, RTOS task).  Never blocks.
 * Returns 0 on success, -1 if the buffer is full (byte is dropped).
 * -------------------------------------------------------------------------- */
static int tx_push(uint8_t byte)
{
    uint16_t next = (txRing.head + 1) % TX_RING_BUF_SIZE;
    if (next == txRing.tail)
        return -1;          // full — drop byte (same policy as your RX ring)

    txRing.buf[txRing.head] = byte;
    txRing.head = next;
    return 0;
}

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
    USART1_Send_ArrayU8(&ch, 1);
}

/* --------------------------------------------------------------------------
 * USART1_Send_ArrayU8  — non-blocking TX via ring buffer + TXE interrupt.
 *
 * Flow:
 *   1. Push all bytes into txRing.
 *   2. If the UART TXE interrupt is not already running, enable it.
 *      The ISR will drain the ring autonomously from here on.
 * -------------------------------------------------------------------------- */
void USART1_Send_ArrayU8(uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0) return;

    /* Push bytes — drop silently if ring is full (same as RX side) */
    for (uint16_t i = 0; i < len; i++)
        tx_push(buf[i]);

    /* Kick the ISR if it is not already running.
     * __HAL_UART_GET_IT_SOURCE checks whether TXE IRQ is already enabled,
     * which means the ISR is still draining — no need to re-enable. */
    if (!__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE))
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
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