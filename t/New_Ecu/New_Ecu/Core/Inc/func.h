/*
 * func.h
 *
 *  Created on: Apr 14, 2025
 *      Author: savva
 */

#ifndef INC_FUNC_H_
#define INC_FUNC_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// --- FUNC Data Structure ---
typedef struct {
    float balance_brake_value;
    uint8_t engine_mode;
    // Add other fields if FUNC provides more data
    uint32_t last_update_time;
} FUNC_Data_t;

// --- Function Prototypes ---
bool FUNC_Init(void);
void FUNC_UpdateData(void);
void FUNC_ProcessMessage(uint8_t* message_data, uint8_t data_len, uint32_t message_id); // If needed

// --- Getter Functions ---
float FUNC_GetBalanceBrakeValue(void);
uint8_t FUNC_GetEngineMode(void);


#endif /* INC_FUNC_H_ */
