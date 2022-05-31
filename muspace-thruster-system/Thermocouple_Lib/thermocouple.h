/*
 * thermocouple.h
 *
 *  Created on: 3 มี.ค. 2565
 *      Author: HEIN.H
 */

#ifndef THERMOCOUPLE_H_
#define THERMOCOUPLE_H_

#include <stm32f0xx_hal.h>
#include <stdio.h>


extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MCP9600_readTemp(uint8_t address,uint8_t pinCfg, float *tempdata);



#endif /* THERMOCOUPLE_H_ */
