/*
 * OBCheader.h
 *
 *  Created on: Apr 22, 2022
 *      Author: parin
 */

#ifndef INC_OBCHEADER_H_
#define INC_OBCHEADER_H_

#define ACU_BUS_ID 0x01

typedef enum
{
	ACU_HANDSHAKE,
	ACU_STARTDETUMBLING,
	ACU_STOPDETUMBLING,
	ACU_GETMODE,
	ACU_GETREPORT,
	ACU_GETANGULARRATE,
	ACU_GETMAGNETIC,
	ACU_GETCURRENT,
	ACU_REQEULER,
	ACU_SETEULER,
	ACU_TOTALCMDCOUNT,
	ACU_SETNEXTARGLEN = 0xA1
}ACU_CommandEnum;

typedef struct
{
	ACU_CommandEnum cmd;
	uint8_t length;
} ACU_cmd_params_type;

static const ACU_cmd_params_type ACU_ReadCmdLength[ACU_TOTALCMDCOUNT] =
{
		{	ACU_HANDSHAKE,				1	},
		{	ACU_STARTDETUMBLING,		1	},
		{	ACU_STOPDETUMBLING,			1	},
		{	ACU_GETMODE,				1	},
		{	ACU_GETREPORT,				1	},
		{	ACU_GETANGULARRATE,			12	},
		{	ACU_GETMAGNETIC,			12	},
		{	ACU_GETCURRENT,				12	},
		{	ACU_REQEULER,				1	},
		{	ACU_SETEULER,				12	},
};

#endif /* INC_OBCHEADER_H_ */
