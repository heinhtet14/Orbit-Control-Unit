/*
 * mpu6050.h
 *
 *  Created on: Apr 11, 2022
 *      Author: HEIN HTET
 *      Description: This is MPU6050 test for alpha version of Lunar Rover VER.1
 */



#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f0xx_hal.h"
#include "string.h"
#include "stdbool.h"
#include <math.h>

//MPU6050 CONF
typedef struct{

	uint8_t ClockSource;
	uint8_t Gyro_Full_Scale;
	uint8_t Acce_Full_Scale;
	uint8_t Config_Low_Pass_Filter;
	bool 	Sleep_Mode;

}MPU6050_Config;

// Clock Source Range
enum Clock_Source_ENUM{
	Internal_8_M_Hz = 0x00
};

// Digital Low Pass Filter
enum DLPF_ENUM{
	DLPF_260A_256G_Hz = 0x00,
	DLPF_5A_5G_Hz 	  = 0x01
};


//Gyroscope FullScale Range (Unit is degree/second)
enum gyro_FullScale_ENUM{
	FS_SEL_250  = 0x00,
	FS_SEL_500  = 0x01,
	FS_SEL_1000 = 0x02,
	FS_SEL_2000 = 0x03
};

//Accelerometer FullScale Range (Unit is 1g = 9.81 m/s2)
enum acce_FullScale_ENUM{
	AFS_SEL_2g  = 0x00,
	AFS_SEL_4g,
	AFS_SEL_8g,
	AFS_SEL_16g
};

// Raw Data
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;

}Raw_Data;

// Scaled Data
typedef struct{
	float x;
	float y;
	float z;
}Scaled_Data;



void MPU6050_Init(I2C_HandleTypeDef *I2C);  // Initialize I2C
void MPU6050_Conf();				      // Configure MPU6050
void MPU6050_Get_Raw_Accel_Data(Raw_Data *raw_data); 	 	 // Get Raw Accelerometer Data
void MPU6050_Get_Raw_Gyro_Data(Raw_Data *raw_data);			//  Get Raw Gyroscope Data
void MPU6050_Scaled_Accel_Data(Scaled_Data *scaled_data); 	// Scaled Accelerometer Data
void MPU6050_Scaled_Gyro_Data(Scaled_Data *scaled_data);	// Scaled Gyroscope Data


#endif /* MPU6050_H_ */
