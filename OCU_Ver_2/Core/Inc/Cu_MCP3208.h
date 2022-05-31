/*
 * Cu_MCP3208.h
 *
 *  Created on: May 5, 2022
 *      Author: jeremy htet
 */

#ifndef INC_CU_MCP3208_H_
#define INC_CU_MCP3208_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h"
#define MCP3208_START_BIT 0x04
#define MCP3208_MODE_SINGLE 0x02

uint16_t Current_Read(SPI_HandleTypeDef *hspi1, uint8_t channel);

#ifdef __cplusplus
}
#endif


#endif /* INC_CU_MCP3208_H_ */
