/*
 * bms.c
 *
 *  Created on: Apr 14, 2025
 *      Author: savva
 */

#include "bms.h"
#include "main.h"      // For peripheral handles (e.g., hcan) if needed
#include <string.h>    // For memset
#include <stdio.h>     // For printf debugging
#include <stdlib.h>    // For rand() if simulating

// --- Define BMS CAN IDs ---
// --- TODO: Replace with your ACTUAL BMS CAN IDs ---
 // // BMS cell voltage data


// --- Private Global Data Variable ---
static BMS_Data_t g_bms_data; // Static makes it private to this file

// --- Initialization ---
bool BMS_Init(void) {

    memset(&g_bms_data, 0, sizeof(BMS_Data_t));

    return true; // Return status
}

// --- Data Update (Called from main loop) ---
void BMS_UpdateData(void) {
    // If the BMS needs periodic requests sent TO it, do it here.
    // Otherwise, if all data comes via CAN Rx interrupts, this can be empty
    // or used for non-CAN related BMS tasks.

    // Example: Request detailed cell voltages periodically if not sent automatically
    /*
    static uint32_t last_cell_volt_request = 0;
    if (HAL_GetTick() - last_cell_volt_request > 1000) { // Request every 1 sec
        last_cell_volt_request = HAL_GetTick();
        // _SendSomeBmsRequestMessage(BMS_REQUEST_CELL_VOLTS); // Your function to send request
    }
    */

    // --- Simulation Code (Keep or remove based on BMS_SIMULATE_DATA) ---
//    #ifdef BMS_SIMULATE_DATA
//    g_bms_data.pack_current = 5.0f + ((float)(rand() % 100) / 50.0f);
//    g_bms_data.pack_inst_voltage = 400.0f + ((float)(rand() % 100) / 10.0f);
//    g_bms_data.pack_soc -= 0.001f; if(g_bms_data.pack_soc < 0) g_bms_data.pack_soc = 0;
//    g_bms_data.high_temp = 35.0f + ((float)(rand() % 100) / 50.0f);
//    g_bms_data.low_temp = 33.0f + ((float)(rand() % 100) / 50.0f);
//    g_bms_data.high_cell_voltage = 4.10f + ((float)(rand() % 100) / 1000.0f);
//    g_bms_data.low_cell_voltage = 4.05f - ((float)(rand() % 100) / 1000.0f);
//    g_bms_data.last_update_time = HAL_GetTick();
//    #endif // BMS_SIMULATE_DATA
}

// --- Message Processing (Called by central CAN Rx Callback) ---
void BMS_ProcessMessage(uint8_t* message_data, uint8_t data_len, uint32_t message_id) {
    // --- TODO: Parse incoming BMS messages based on ID and expected data format ---
    // Apply scaling factors and offsets specific to your BMS.
	extern BMS_Data_t g_bms_data; // Access global BMS data structure

    // Update timestamp regardless of message content
    //g_bms_data.last_update_time = HAL_GetTick();

    switch(message_id) {
        case BMS_ID_VOLT:
        	g_bms_data.high_cell_voltage = (float)((uint16_t)(message_data[0] | (message_data[1] << 8))) * .0001f;
        	g_bms_data.low_cell_voltage = (float)((uint16_t)(message_data[2] | (message_data[3] << 8))) * .0001f;
        	g_bms_data.pack_summed_voltage = (float)((uint16_t)(message_data[4] | (message_data[5] << 8))) * .01f;
        	break;

        case BMS_ID_MAIN:
            g_bms_data.pack_current = (float)((int16_t)(message_data[0] | (message_data[1] << 8)));
      		g_bms_data.pack_inst_voltage = (float)((uint16_t)(message_data[2] | (message_data[3] << 8))) * .1f;
            g_bms_data.pack_soc = (float)((uint8_t)(message_data[4])) / 2; // Example SOC scaling
            break;

        case BMS_ID_LIMITS:
             g_bms_data.pack_dcl = (float)((int16_t)(message_data[0] | (message_data[1] << 8)));
             g_bms_data.max_pack_voltage = (float)((uint16_t)(message_data[2] | (message_data[3] << 8))) * .1f;
             break;

         case BMS_ID_TEMP:
        	 g_bms_data.high_temp = (float)((int16_t)(message_data[0] | (message_data[1] << 8)));
        	 g_bms_data.low_temp = (float)((int16_t)(message_data[2] | (message_data[3] << 8)));

        	 g_bms_data.max_pack_dcl = (float)((int16_t)(message_data[4] | (message_data[5] << 8)));
             break;

        // Add cases for ALL other relevant BMS CAN IDs

        default:
            // This function should only be called with IDs filtered for BMS,
            // but a default case can catch unexpected IDs if filtering is broad.
            // printf("WARN: BMS_ProcessMessage called with unexpected ID 0x%lX\n", message_id);
            break;
    }
}

/**
 * @brief  Packages and transmits all data from g_bms_data to a specified CAN bus.
 * @note   This function mirrors the data structure and scaling received from CAN1.
 *         It unconditionally attempts to queue each message for transmission.
 *         If transmit mailboxes are full, messages will be silently dropped.
 * @param  hcan: Pointer to the CAN handle to transmit on (e.g., &hcan2).
 * @retval None
 */
void BMS_SendAllDataToCan2(CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox; // Still required by the HAL function signature

    // --- Common Header Settings ---
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;


    // --- 1. Send BMS_ID_MAIN Message (ID: 0x183) ---
    tx_header.StdId = BMS_ID_MAIN;
    tx_header.DLC = 5;

    // Repack data by reversing the parsing logic
    int16_t pack_current_raw      = (int16_t)g_bms_data.pack_current;
    uint16_t pack_inst_voltage_raw = (uint16_t)(g_bms_data.pack_inst_voltage / 0.1f);
    uint8_t pack_soc_raw          = (uint8_t)(g_bms_data.pack_soc * 2.0f);

    tx_data[0] = (uint8_t)(pack_current_raw & 0xFF);
    tx_data[1] = (uint8_t)(pack_current_raw >> 8);
    tx_data[2] = (uint8_t)(pack_inst_voltage_raw & 0xFF);
    tx_data[3] = (uint8_t)(pack_inst_voltage_raw >> 8);
    tx_data[4] = pack_soc_raw;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);


    // --- 2. Send BMS_ID_VOLT Message (ID: 0x182) ---
    tx_header.StdId = BMS_ID_VOLT;
    tx_header.DLC = 6;

    // Repack data
    uint16_t high_cell_volt_raw   = (uint16_t)(g_bms_data.high_cell_voltage / 0.0001f);
    uint16_t low_cell_volt_raw    = (uint16_t)(g_bms_data.low_cell_voltage / 0.0001f);
    uint16_t pack_summed_volt_raw = (uint16_t)(g_bms_data.pack_summed_voltage / 0.01f);

    tx_data[0] = (uint8_t)(high_cell_volt_raw & 0xFF);
    tx_data[1] = (uint8_t)(high_cell_volt_raw >> 8);
    tx_data[2] = (uint8_t)(low_cell_volt_raw & 0xFF);
    tx_data[3] = (uint8_t)(low_cell_volt_raw >> 8);
    tx_data[4] = (uint8_t)(pack_summed_volt_raw & 0xFF);
    tx_data[5] = (uint8_t)(pack_summed_volt_raw >> 8);

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);


    // --- 3. Send BMS_ID_LIMITS Message (ID: 0x184) ---
    tx_header.StdId = BMS_ID_LIMITS;
    tx_header.DLC = 4;

    // Repack data
    int16_t pack_dcl_raw        = (int16_t)g_bms_data.pack_dcl;
    uint16_t max_pack_volt_raw = (uint16_t)(g_bms_data.max_pack_voltage / 0.1f);

    tx_data[0] = (uint8_t)(pack_dcl_raw & 0xFF);
    tx_data[1] = (uint8_t)(pack_dcl_raw >> 8);
    tx_data[2] = (uint8_t)(max_pack_volt_raw & 0xFF);
    tx_data[3] = (uint8_t)(max_pack_volt_raw >> 8);

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);


    // --- 4. Send BMS_ID_TEMP Message (ID: 0x185) ---
    tx_header.StdId = BMS_ID_TEMP;
    tx_header.DLC = 6;

    // Repack data
    int16_t high_temp_raw      = (int16_t)g_bms_data.high_temp;
    int16_t low_temp_raw       = (int16_t)g_bms_data.low_temp;
    int16_t max_pack_dcl_raw   = (int16_t)g_bms_data.max_pack_dcl;

    tx_data[0] = (uint8_t)(high_temp_raw & 0xFF);
    tx_data[1] = (uint8_t)(high_temp_raw >> 8);
    tx_data[2] = (uint8_t)(low_temp_raw & 0xFF);
    tx_data[3] = (uint8_t)(low_temp_raw >> 8);
    tx_data[4] = (uint8_t)(max_pack_dcl_raw & 0xFF);
    tx_data[5] = (uint8_t)(max_pack_dcl_raw >> 8);

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// --- Getter Functions (Keep as before) ---
float BMS_GetPackCurrent(void) { return g_bms_data.pack_current; }
float BMS_GetPackInstVoltage(void) { return g_bms_data.pack_inst_voltage; }
float BMS_GetPackSOC(void) { return g_bms_data.pack_soc; }
float BMS_GetPackDCL(void) { return g_bms_data.pack_dcl; }
float BMS_GetHighTemp(void) { return g_bms_data.high_temp; }
float BMS_GetLowTemp(void) { return g_bms_data.low_temp; }
float BMS_GetPackSummedVoltage(void) { return g_bms_data.pack_summed_voltage; }
float BMS_GetHighCellVoltage(void) { return g_bms_data.high_cell_voltage; }
float BMS_GetLowCellVoltage(void) { return g_bms_data.low_cell_voltage; }
float BMS_GetMaxPackDCL(void) { return g_bms_data.max_pack_dcl; }
float BMS_GetMaxPackVoltage(void) { return g_bms_data.max_pack_voltage; }
