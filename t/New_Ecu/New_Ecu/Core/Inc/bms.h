/*
 * bms.h
 *
 *  Created on: Apr 13, 2025
 *      Author: savva
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include "stm32f4xx_hal.h" // Or appropriate HAL header
#include <stdint.h>
#include <stdbool.h>

#define BMS_ID_VOLT    		0x182
// Main BMS data
#define BMS_ID_MAIN			0x183
// BMS limits data
#define BMS_ID_LIMITS       0x184
// BMS temperature data
#define BMS_ID_TEMP			0x185

// --- Configuration ---
//#define BMS_SIMULATE_DATA // Uncomment to enable simulated data generation

// --- BMS Data Structure ---
typedef struct {
    // Fields matching shared_spi_data.h (with bms_ prefix removed for internal use)
    float pack_current;
    float pack_inst_voltage;
    float pack_soc;
    float pack_dcl;
    float high_temp;
    float low_temp;
    float pack_summed_voltage;
    float high_cell_voltage;
    float low_cell_voltage;
    float max_pack_dcl;
    float max_pack_voltage;
    // Add other relevant fields if needed
    uint32_t last_update_time; // Optional: timestamp of last update
} BMS_Data_t;

// --- Function Prototypes ---
bool BMS_Init(void); // Add parameters if needed (e.g., CAN_HandleTypeDef*)
void BMS_UpdateData(void);
void BMS_ProcessMessage(uint8_t* message_data, uint8_t data_len, uint32_t message_id); // Adapt params as needed
void BMS_SendAllDataToCan2(CAN_HandleTypeDef *hcan);
// --- Getter Functions ---
float BMS_GetPackCurrent(void);
float BMS_GetPackInstVoltage(void);
float BMS_GetPackSOC(void);
float BMS_GetPackDCL(void);
float BMS_GetHighTemp(void);
float BMS_GetLowTemp(void);
float BMS_GetPackSummedVoltage(void);
float BMS_GetHighCellVoltage(void);
float BMS_GetLowCellVoltage(void);
float BMS_GetMaxPackDCL(void);
float BMS_GetMaxPackVoltage(void);

#endif /* INC_BMS_H_ */
