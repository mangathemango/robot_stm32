#pragma once
#include "main.h"

void Beep_Start(uint32_t duration_ms);
void Beep_Update(void);  // call this in Bsp_Loop, that's the ONLY thing in main