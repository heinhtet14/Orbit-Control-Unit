/*
 * FLASH_MEM.c
 *
 *  Created on: Apr 25, 2022
 *      Author: HEIN.H
 */
#include "FLASH_MEM.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"


//Flash Address Sector Selection
static uint32_t GetSector(uint32_t Address){
	uint32_t sector = 0;

	if((Address < 0x08003FFF) && (Address >=  0x08000000))
	{
		sector = FLASH_SECTOR_0;
	}
	if((Address <  0x08007FFF) && (Address >=  0x08004000))
	{
		sector = FLASH_SECTOR_1;
	}
	if((Address < 0x0800BFFF) && (Address >=  0x08008000))
	{
		sector = FLASH_SECTOR_2;
	}
	if((Address < 0x0800FFFF) && (Address >=  0x0800C000))
	{
		sector = FLASH_SECTOR_3;
	}
	if((Address < 0x0801FFFF) && (Address >=  0x08010000))
	{
		sector = FLASH_SECTOR_4;
	}
	if((Address < 0x0803FFFF) && (Address >=  0x08020000))
	{
		sector = FLASH_SECTOR_5;
	}
	if((Address < 0x0805FFFF) && (Address >=  0x08040000))
	{
		sector = FLASH_SECTOR_6;
	}
	if((Address < 0x0807FFFF) && (Address >=  0x08060000))
	{
		sector = FLASH_SECTOR_7;
	}

	return sector;
}

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar = 0;
	int num_of_words = (strlen(Data)/4) + ((strlen(Data)%4)!= 0);

	/*Unlock Flash Memory to enable the flash control register access*/
	HAL_FLASH_Unlock();

	/*Erase the Flash*/
	/*Get the number of sector to erase from 1st sector*/
	uint32_t StartSector = GetSector(StartSectorAddress);
	uint32_t EndSectorAddress = StartSectorAddress + num_of_words*4;
	uint32_t EndSector = GetSector(EndSectorAddress);

	/*Fill EraseInit Structure*/

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector    = StartSector;
	EraseInitStruct.NbSectors = (EndSector - StartSector)+1;

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError();
	}
	while (sofar<num_of_words){
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, Data[sofar]) == HAL_OK)
		{
			StartSectorAddress += 4;
			sofar ++;

		}
		else {
			return HAL_FLASH_GetError();
		}

}
	HAL_FLASH_Lock();
	return 0;
}

void Flash_Read_Data(uint32_t StartSectorAddress,uint32_t *Data){

	while(1){
		*Data = *(uint32_t *)StartSectorAddress;
		if(*Data == 0xffffff){
			*Data = '\0';
			break;
		}
		StartSectorAddress += 4;
		Data++;
	}
}

void Convert_To_String(uint32_t *Data, char *str){
	int num_of_bytes = ((strlen(Data)/4) + ((strlen(Data)%4)!= 0)) *4;
	for (int i =0; i<num_of_bytes; i++){
		str[i] = Data[i/4]>>(8*(i%4));
	}
}
