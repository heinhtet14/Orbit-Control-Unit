/*
 * Vo_MCP3208.h
 *
 *  Created on: May 5, 2022
 *      Author: HEIN.H
 */

#ifndef INC_VO_MCP3208_H_
#define INC_VO_MCP3208_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h"
#define MCP3208_START_BIT 0x04
#define MCP3208_MODE_SINGLE 0x02

uint16_t Voltage_Read(SPI_HandleTypeDef *hspi3, uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* INC_VO_MCP3208_H_ */
