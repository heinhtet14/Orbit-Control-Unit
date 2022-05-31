/*
 * adc_dac.h
 *
 *  Created on: Mar 4, 2022
 *      Author: HEIN.H
 */

#ifndef ADC_DAC_H_
#define ADC_DAC_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f0xx_hal.h"

void ADC_System_Start(ADC_HandleTypeDef *hadc, uint32_t *values,
		TIM_HandleTypeDef *timer);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif
#endif /* ADC_DAC_H_ */
