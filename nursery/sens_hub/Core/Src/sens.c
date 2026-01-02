#include "main.h"
#include "sens.h"


uint8_t tds_flag = 0; 		 // adc1 업데이트 플래그
uint8_t ph_flag = 0;  		// adc2 업데이트 플래그
uint32_t tds_value;
uint32_t ph_value;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) tds_value = HAL_ADC_GetValue(hadc);
  if (hadc->Instance == ADC2) ph_value =HAL_ADC_GetValue(hadc);
}



