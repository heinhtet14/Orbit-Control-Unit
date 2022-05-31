/*
 * adc_dac.c
 *
 *  Created on: Mar 4, 2022
 *      Author: HEIN.H
 */
#include "adc_dac.h"
#include <stm32f0xx_hal.h>

void ADC_System_Start(ADC_HandleTypeDef *hadc, uint32_t *values,
		TIM_HandleTypeDef *timer) {

	HAL_TIM_Base_Start(timer);
	HAL_ADC_Start_DMA(hadc, values, 3);


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

}
