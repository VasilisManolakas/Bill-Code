/*
 * FIRFilter.h
 *
 *  Created on: March 27, 2025
 *      Author: Savvas Spyridonidis
 *      Reviewed on August 16, 2025 by Vasilis Manolakas
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include <stdint.h>

#define FIR_FILTER_LENGTH 16	//FIR Digital Filter looks to the last 16 samples

typedef struct {
	float buf[FIR_FILTER_LENGTH];	//Buffer that holds the 16 recent inputs
	uint8_t bufIndex;		//Buffer index  (where the next sample will be stored (0-15))
	float out;					//last output
} FIRFilter;
void FIRFilter_Init(FIRFilter *fir);	//FIR Initialization
float FIRFilter_Update (FIRFilter *fir, float inp);	//FIR Computing

#endif /* INC_FIRFILTER_H_ */
