/*
 * GATE.c
 *
 *  Created on: May 5, 2022
 *      Author: HEIN.H
 */
#include "GATE.h"
#include "stm32h7xx_hal.h"

void Sensors_Init() /* Initializing all the overall subsystems */
{
	void Thurster_Head_On();
	void Solenoid_On_State();
	void PFCV_On_State();
	void Cathode_Heater_On();
	void Cathode_Keeper_On();


}
void Thurster_Head_On()
{
	HAL_GPIO_WritePin(Thurster_Head_Port,Thurster_Head_Pin, OFF);

}
void Thurster_Head_Off()
{
	HAL_GPIO_WritePin(Thurster_Head_Port,Thurster_Head_Pin, ON);

}
void Solenoid_On_State()
{
	HAL_GPIO_WritePin(Solenoid_Port,Solenoid_Pin, ON);

}
void PFCV_On_State()
{
	HAL_GPIO_WritePin(PFCV_Port,PFCV_Pin, ON);

}
void Cathode_Heater_On()
{
	HAL_GPIO_WritePin(Cathode_Heater_Port, Cathode_Heater_Pin, ON);

}
void Cathode_Heater_Off()
{
	HAL_GPIO_WritePin(Cathode_Heater_Port, Cathode_Heater_Pin, OFF);
}
void Cathode_Keeper_On()
{
	HAL_GPIO_WritePin(Cathode_Keeper_Port,Cathode_Keeper_Pin, ON);

}
void Cathode_Keeper_Off()
{
	HAL_GPIO_WritePin(Cathode_Keeper_Port,Cathode_Keeper_Pin, OFF);

}

