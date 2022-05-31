/*
 * max6675.h
 *
 *  Created on: Feb 24, 2022
 *      Author: HEIN.H
 */

#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_

#include "stm32f0xx_hal.h"
#include "stdbool.h"

#define SPI_TIMEOUT 1000

typedef struct{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* Thx_CS_Port;
	uint16_t Thx_CS_Pin;
	uint16_t Thx_rawdata;
	uint16_t Thx_celcius;
	bool connected;

}ThermoCouple;

void ReadThermoCouple(ThermoCouple *Th);



#endif /* INC_MAX6675_H_ */
