/*
 * thermocouple.c
 *
 *  Created on: 3 มี.ค. 2565
 *      Author: HEIN.H
 */
#include "thermocouple.h"
#include <stm32f0xx_hal.h>

/*
HAL_StatusTypeDef MCP9600_readTemp(uint8_t address,uint8_t pinCfg, float *tempdata)
{
	HAL_StatusTypeDef ret;
	uint8_t addata[2];

	ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(address<<1), &pinCfg, 1, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	for(int i = 0; i< 5000; i++);

	ret = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(address<<1)|0x01, addata, 2, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	*tempdata = ((addata[0] & 0x0F)<< 8) | addata[1];
	return HAL_OK;
}
//uint8_t buff[0] = Regular_Temp;
//HAL_StatusTypeDef ret;


}
*/


