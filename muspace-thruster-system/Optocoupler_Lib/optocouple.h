/*
 * optocouple.h
 *
 *  Created on: Feb 28, 2022
 *      Author: HEIN.H
 */

#ifndef OPTOCOUPLE_H_
#define OPTOCOUPLE_H_

#define ON						 1
#define OFF						 0

#define Thurster_Head_Port		 GPIOC
#define Thurster_Head_Pin 		 GPIO_PIN_8

#define Solenoid_Port			 GPIOC
#define Solenoid_Pin			 GPIO_PIN_6

#define PFCV_Port 				 GPIOC
#define PFCV_Pin				 GPIO_PIN_5

#define Cathode_Heater_Port		 GPIOA
#define Cathode_Heater_Pin		 GPIO_PIN_12

#define Cathode_Keeper_Port		 GPIOA
#define Cathode_Keeper_Pin		 GPIO_PIN_11


#include <stm32f0xx.h>
#include <stm32f0xx_hal.h>

void Sensors_Init(); /* Initialize Sensor Ports */
void Thurster_Head_On_State();
void Thurster_Head_Off_State();
void Solenoid_On_State();
void Solenoid_Off_State();
void PFCV_On_State();
void PFCV_Off_State();
void Cathode_Heater_On_State();
void Cathode_Heater_Off_State();
void Cathode_Keeper_Relay_On_State();
void Cathode_Keeper_Relay_Off_State();







#endif /* OPTOCOUPLE_H_ */
