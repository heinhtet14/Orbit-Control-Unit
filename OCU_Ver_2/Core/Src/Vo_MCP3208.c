/*
 * Vo_MCP3208.c
 *
 *  Created on: May 5, 2022
 *      Author: HEIN.H
 */
#include "Vo_MCP3208.h"
#include <stm32h7xx_hal.h>
#include <main.h>

uint16_t Voltage_Read(SPI_HandleTypeDef *hspi3, uint8_t channel){
	uint8_t aTxBuffer[3];
	uint8_t aRxBuffer[3];

	aTxBuffer[0] = MCP3208_START_BIT | MCP3208_MODE_SINGLE | ((channel & 0x04) >> 2);
	aTxBuffer[1] = channel << 6;
	aTxBuffer[2] = 0x00;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	if(HAL_SPI_TransmitReceive(hspi3,aTxBuffer,aRxBuffer,3,100) != HAL_OK){
			Error_Handler();
	}
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	uint16_t conv_result = ((uint16_t)(aRxBuffer[1] & 0x0f) << 8) | (uint16_t)aRxBuffer[2];

	return conv_result;

}

