/*
 * mpu6050.c
 *
 *  Created on: Apr 11, 2022
 *      Author: HEIN.H
 */
#include "mpu6050.h"

static I2C_HandleTypeDef i2cHandler; // I2C Handle


void MPU6050_Init(I2C_HandleTypeDef *I2C){
	memcpy(&i2cHandler, I2C, sizeof(*I2C));
}

