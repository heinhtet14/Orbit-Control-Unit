/*
 * FIR_DSP.h
 *
 *  Created on: Apr 4, 2022
 *      Author: HEIN.H
 */

#ifndef FIR_DSP_H_
#define FIR_DSP_H_

#include <stdint.h>

#define FIR_FILTER_LENGTH 16
typedef struct{

	float buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;

}FIRFILTER;

void FIR_DSP_Init(FIRFILTER *fir); //FIR Filter Initialization
float FIR_DSP_Update(FIRFILTER *fir, float inp);


#endif /* FIR_DSP_H_ */
