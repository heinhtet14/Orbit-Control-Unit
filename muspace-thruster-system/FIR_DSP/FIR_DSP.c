/*
 * FIR_DSP.c
 *
 *  Created on: Apr 4, 2022
 *      Author: HEIN.H
 */
#include "FIR_DSP.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,0.0254209f,0.0724719f,0.1311260f,0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};

void FIR_DSP_Init(FIRFILTER *fir){
	/*Clear filter buffer*/

	for(uint8_t n = 0; n < FIR_FILTER_LENGTH; n++){
		fir->buf[n] = 0.0f;
	}

	/* Reset Buffer Index */
	fir->bufIndex = 0;

	/*Clear filter output */
	fir->out = 0.0f;

}

float FIR_DSP_Update(FIRFILTER *fir, float inp) {
	/*Store latest sample in buffer*/

	fir -> buf[fir->bufIndex] = inp; //circular buffer index is pointing to in the circular buffer

	/*Increment the buffer index */
	fir -> bufIndex++;

	if(fir->bufIndex == FIR_FILTER_LENGTH){
		fir->bufIndex = 0;
	}

	/* Compute new output signal convolution */
	fir -> out = 0.0f;
	uint8_t sumIndex = fir->bufIndex;

	for(uint8_t n=0; n < FIR_FILTER_LENGTH; n++) {
		if (sumIndex > 0){
			sumIndex--;
		} else {
			sumIndex = FIR_FILTER_LENGTH - 1;
		}

		/* Real Equation */
		fir->out += FIR_IMPULSE_RESPONSE[n]* fir->buf[sumIndex];
	}

	return fir->out; // return output
}

