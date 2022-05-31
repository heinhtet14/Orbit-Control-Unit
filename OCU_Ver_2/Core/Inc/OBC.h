/*
 * OBC.h
 *
 *  Created on: May 5, 2022
 *      Author: jeremy
 */

#ifndef INC_OBC_H_
#define INC_OBC_H_

#define OCU_BUS_ID 0x01

typedef enum
{
	OCU_HANDSHAKE,
	OCU_WARMUP,
	OCU_FIRE_THRUSTER,
	OCU_SHUTDOWN_THRUSTER,
	OCU_GET_MODE,
	OCU_GET_REPORT,
	OCU_GET_TEMPERATURE,
	OCU_PRESSURE_DATA,
	OCU_REMAINING_KRYPTON,
	OCU_GET_CURRENT,
	OCU_GET_VOLTAGE,
	OCU_GET_COMMAND_COUNT,
	OCU_SETNEXTTARGLEN = 0xA1
}OCU_CommandEnum;

typedef struct
{
	OCU_CommandEnum cmd;
	uint8_t length;
}OCU_cmd_params_type;

static const OCU_cmd_params_type OCU_ReadCmdLength[OCU_GET_COMMAND_COUNT] =
{
		{OCU_HANDSHAKE,						1},
		{OCU_WARMUP,						1},
		{OCU_FIRE_THRUSTER,					1},
		{OCU_SHUTDOWN_THRUSTER,				1},
		{OCU_GET_MODE,						1},
		{OCU_GET_REPORT,					1},
		{OCU_GET_TEMPERATURE,				1},
		{OCU_PRESSURE_DATA,					1},
		{OCU_REMAINING_KRYPTON,				1},
		{OCU_GET_CURRENT,					1},
		{OCU_GET_VOLTAGE,					1},


};

#endif /* INC_OBC_H_ */
