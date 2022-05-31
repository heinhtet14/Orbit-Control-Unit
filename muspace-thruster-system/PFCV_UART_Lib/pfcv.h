/*
 * pfcv.h
 *
 *  Created on: Mar 4, 2022
 *      Author: HEIN.H
 */

#ifndef PFCV_H_
#define PFCV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PFCV_TX_BUFFER 6
#define PFCV_RX_BUFFER 49

double pfcv_init(UART_HandleTypeDef *huart, uint8_t *tx, uint8_t *rx
		,uint8_t *mass_control);
void pfcv_send_set_point(UART_HandleTypeDef *uart3, int temp, uint8_t *tx, uint32_t adcvalue);



#ifdef __cplusplus
}
#endif

#endif /* PFCV_H_ */
