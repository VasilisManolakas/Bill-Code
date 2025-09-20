/*
 * func.c
 *
 *  Created on: Apr 14, 2025
 *      Author: savva
 */


#include "func.h"
#include "main.h"

// --- Private Global Data Variable ---
static FUNC_Data_t g_func_data;

// --- Define FUNC CAN IDs ---
#define FUNC_ID 0x1 // Example ID for FUNC data

// --- Initialization ---
bool FUNC_Init(void) {
    // TODO: Initialize communication interface for func
    // (Could be CAN, UART, Digital I/O, etc.)

    memset(&g_func_data, 0, sizeof(FUNC_Data_t));
    printf("FUNC Initialized\n");
    return true;
}

// --- Data Update ---
void FUNC_UpdateData(void) {
    // TODO: Get new data from func source
    // Example: Read from specific CAN messages, calculate values...
    // g_func_data.balance_brake_value = Calculate_Brake_Balance();
    // g_func_data.engine_mode = Read_Engine_Mode_Switch(); // Read GPIO?

    g_func_data.last_update_time = HAL_GetTick();
}

// --- Message Processing ---
void FUNC_ProcessMessage(uint8_t* message_data, uint8_t data_len, uint32_t message_id) {
    extern FUNC_Data_t func_data;

    if (message_data == NULL || data_len == 0) {
		return; // Safety check
	}

    if (message_id == FUNC_ID){
    	// from 0-11 value
    	func_data.balance_brake_value = (float)((uint16_t)(message_data[0] | (message_data[1] << 8))) / 10.0f;
    	// Extract engine mode (3rd byte)
    	func_data.engine_mode = message_data[2];

    	// Extract rotary switch value for stepper motor positioning (4th byte)
    	// Update the global rotary switch value


    }
    // Process incoming CAN messages for func


}

// --- Getter Functions ---
float FUNC_GetBalanceBrakeValue(void) {
    return g_func_data.balance_brake_value;
}
uint8_t FUNC_GetEngineMode(void) {
    return g_func_data.engine_mode;
}
