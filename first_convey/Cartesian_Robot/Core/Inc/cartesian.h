#ifndef CARTESIAN_H
#define CARTESIAN_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

void Cartesian_Task(void);

void Cartesian_SetXStepsPerMM(float steps_per_mm);

void Cartesian_SetXRefCm(float ref_cm);

void Cartesian_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);
void Cartesian_OnUartRxCplt(UART_HandleTypeDef *huart);

bool Cartesian_IsBusy(void);

#endif
