/*
 * max6675
 *
 *  Created on: Feb 24, 2022
 *      Author: HEIN.H
 */
#define TRUE 1
#define FALSE 0
#define bool BYTE
#include "max6675.h"

static uint8_t SPI_Rx(SPI_HandleTypeDef *hspi){
	uint8_t data, data0;
	data = 0xFF;
	data0 = 0;

	while ((HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY));
	HAL_SPI_TransmitReceive(hspi, &data, &data0, 1, SPI_TIMEOUT);
	return data0;
}

void ReadThermoCouple(ThermoCouple *Th) {
	unsigned short data;
	HAL_GPIO_WritePin(Th -> Thx_CS_Port, Th -> Thx_CS_Pin, GPIO_PIN_RESET );
	data = SPI_Rx(Th -> hspi);
	data <<= 8;
	data |= SPI_Rx(Th -> hspi);
	HAL_GPIO_WritePin(Th -> Thx_CS_Port, Th -> Thx_CS_Pin, GPIO_PIN_SET);
	Th -> Thx_rawdata = data;
	if ( data & 4 ) Th -> connected = FALSE;
	else Th -> connected = TRUE;
	data >>=3;
	Th -> Thx_celcius = data*0.25;

}


