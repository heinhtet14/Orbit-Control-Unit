/*
 * optocouple.c
 *
 *  Created on: Feb 28, 2022
 *      Author: HEIN.H
 */
#include "optocouple.h"
#include "thermocouple.h"

void Sensors_Init() /* Initializing all the overall subsystems */
{
	void Thurster_Head_On_State();
	void Solenoid_On_State();
	void PFCV_On_State();
	void Cathode_Heater_On_State();
	void Cathode_Keeper_Relay_On_State();


}
void Thurster_Head_On_State()
{
	HAL_GPIO_WritePin(Thurster_Head_Port,Thurster_Head_Pin, OFF);

}
void Thurster_Head_Off_State()
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
void Cathode_Heater_On_State()
{
	HAL_GPIO_WritePin(Cathode_Heater_Port, Cathode_Heater_Pin, ON);

}
void Cathode_Keeper_Relay_On_State()
{
	HAL_GPIO_WritePin(Cathode_Keeper_Port,Cathode_Keeper_Pin, OFF);

}





//}

