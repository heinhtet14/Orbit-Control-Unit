/*
 * FLASH_MEM.h
 *
 *  Created on: Apr 25, 2022
 *      Author: HEIN.H
 */

#ifndef FLASH_MEM_H_
#define FLASH_MEM_H_

#include "stdint.h"

uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data);// For PFCV

void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *Data);

void Convert_To_String (uint32_t *Data, char *Str);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num); // For Voltage&Current Values

float Flash_Read_NUM (uint32_t StartSectorAddress);


#endif /* FLASH_MEM_H_ */
