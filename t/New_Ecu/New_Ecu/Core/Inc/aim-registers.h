/*
 * aim-registers.h
 *
 *  Created on: Sep 7, 2025
 *      Author: Vasilis Manolakas
 */
#ifndef AIM_REGISTERS_H
#define AIM_REGISTERS_H
#include "shared_spi_data.h"
#include "bamocar-can.h"  // for BamocarData
//Note: This header file is intended to provide basic declarations for our AiM 1.2 MxS Data Logger's Telemetry:
//We wanted to transmit Bamocar data to our Data Logger through CAN, as requested by our team's ESO.
//Note : Selected the 0x500 memory block because it does not interfere/overlap with our other CAN IDs, e.g. our 0x180 block.
//We will use CAN2 for our data logger.
// CAN IDs reserved for telemetry to the data logger

#define AiM_CAN_ID_PERFORMANCE      0x500   // rpm, torque, voltage
#define AiM_CAN_ID_TEMPS     		0x501   // motor & controller temps
#define AiM_CAN_ID_FLAGS     		0x502   // status flags (brake, R2D, fan, pump)

#define AiM_CAN_ID_BAMO_CURR_A   0x503   // I_Q_CMD, I_Q_CMD_RAMP, I_ACTUAL, I_Q_ACTUAL
#define AiM_CAN_ID_BAMO_CURR_B   0x504   // I1_ACTUAL, I2_ACTUAL, I3_ACTUAL, I_ACTUAL_FILT
#define AiM_CAN_ID_BAMO_FOC      0x505   // I_D_ACTUAL, I_Q_ERROR, I_D_ERROR, N_ACTUAL_FILT
#define AiM_CAN_ID_BAMO_VOLT     0x506   // V_D, V_EMF, V_Q, V_DC_ACTUAL
//Data Logging CAN Tx Headers Initialization function declaration
#define AiM_CAN_ID_APPS 0x507 //APPS Signals
#define AiM_CAN_ID_MAPPED_BAMOCAR_TORQUE_VALUE 0x508	//desired_torque_to_bamocar
void AiM_Initialize_CAN_Tx_Headers(CAN_HandleTypeDef *hcan);

//CAN Send Performance Data Function initialization
void LoggerCAN_Send_Performance_Data(const CombinedLogData_t *perf);
//CAN Send temperature data function initialization
void LoggerCAN_SendTemps(const CombinedLogData_t *tmps);
//CAN Send flag data function initialization
void LoggerCAN_SendFlags(const CombinedLogData_t *flgs);
void LoggerCAN_Send_APPS(const CombinedLogData_t *d);

// --- Bamocar extra transmit functions for AiM logger ---
void AiM_CAN_Send_Temperature_Data(const CombinedLogData_t *d);
void AiM_CAN_Send_BamocarCurrentsA(const BamocarData *bd);
void AiM_CAN_Send_BamocarCurrentsB(const BamocarData *bd);
void AiM_CAN_Send_BamocarFOC     (const BamocarData *bd);
void AiM_CAN_Send_BamocarVoltages(const BamocarData *bd);
void AiM_CAN_Send_Mapped_Torque_Value(const BamocarData *bd);

#endif

