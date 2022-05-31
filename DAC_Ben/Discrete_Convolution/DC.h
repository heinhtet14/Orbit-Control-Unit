/*
 * DC.h
 *
 *  Created on: Apr 22, 2022
 *      Author: HEIN.H
 */

#ifndef DC_H_
#define DC_H_
#include <stdint.h>

#define DISCRETE_CONV 301
typedef struct{
	float buf[301];
	uint8_t bufIndex;
	float y_out;
}DC;

void DC_Init(DC *dc);
float DC_Update(DC *dc, float x_in);

#endif /* DC_H_ */
