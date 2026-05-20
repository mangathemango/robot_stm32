#include "beep.h"
#include "main.h"

static uint32_t beepEndTime = 0;
static uint8_t  beeping     = 0;

void Beep_Start(uint32_t duration_ms)
{
    if(beeping) return;

    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    beepEndTime = HAL_GetTick() + duration_ms;
    beeping = 1;
}

void Beep_Update(void)
{
    if (beeping && HAL_GetTick() >= beepEndTime)
    {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        beeping = 0;
    }
}