/*
 * FIRFilter.c
 *
 *  Created on: March 27, 2025
 *      Author: Savvas Spyridonidis
 *      Reviewed on August 16, 2025 by Vasilis Manolakas
 */


#include "FIRFilter.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0033731f,-0.0039134f,-0.0026050f,0.0077539f,0.0338131f,0.0757223f,0.1242202f,0.1634660f,0.1785714f,0.1634660f,0.1242202f,0.0757223f,0.0338131f,0.0077539f,-0.0026050f,-0.0039134f};

void FIRFilter_Init (FIRFilter *fir) {
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++)	//16 iterations  (FIR_FILTER_LENGTH = 16)
		fir->buf [n] = 0.0f;	//Clear previous data
	fir->bufIndex = 0;		//Reset Buffer Index Pointer so that the next sample will be written at buf[0]
	fir->out = 0.0f;	//Last Output is set to 0.
}


float FIRFilter_Update(FIRFilter *fir, float inp){

	fir->buf[fir->bufIndex] = inp;	//The Newest Input is stored in the buffer at the current index position

	fir->bufIndex++;	//Buffer index is incremented by 1

	if(fir->bufIndex == FIR_FILTER_LENGTH){
		fir->bufIndex=0;	//If the buffer index is equal to 16 in this case, reset it for later inputs.
	}

	fir->out = 0.0f;	//Set last value to 0

	uint8_t sumIndex = fir->bufIndex;	//Set a sum index to the next place in memory to write a sample

	for(uint8_t n=0; n<FIR_FILTER_LENGTH; n++){	//Compute new FIR value after filter

		if(sumIndex > 0){
			sumIndex--;
		} else {
			sumIndex = FIR_FILTER_LENGTH - 1;
		}

		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}
	return fir->out;  //return the new value computed from the last 16 inputs
}
