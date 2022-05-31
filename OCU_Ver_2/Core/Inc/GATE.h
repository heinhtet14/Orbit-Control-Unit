/*
 * GATE.h
 *
 *  Created on: May 5, 2022
 *      Author: HEIN.H
 */

#ifndef INC_GATE_H_
#define INC_GATE_H_

#ifdef __cplusplus
extern "C"{
#endif

#define ON 			1
#define OFF			0

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



#include <stm32h7xx_hal.h>

void Sensors_Init(); /* Initialize Sensor Ports */
void Thurster_Head_On();
void Thurster_Head_Off();
void Solenoid_On_State();
void Solenoid_Off_State();
void PFCV_On_State();
void PFCV_Off_State();
void Cathode_Heater_On();
void Cathode_Heater_Off();
void Cathode_Keeper_On();
void Cathode_Keeper_Off();

#ifdef __cplusplus
}
#endif

#endif /* INC_GATE_H_ */
