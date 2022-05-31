/*
 * PFCV.h
 *
 *  Created on: May 5, 2022
 *      Author: HEIN.H
 */

#ifndef INC_PFCV_H_
#define INC_PFCV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
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

#endif /* INC_PFCV_H_ */
