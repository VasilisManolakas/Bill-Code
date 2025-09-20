/*
 * bamocar-can.h
 *
 *  Created on: Oct 24, 2024
 *      Author: savva
 */

#ifndef BAMOCAR_CAN_H
#define BAMOCAR_CAN_H

#include "stm32f4xx_hal.h"
#include "bamocar-registers.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CAN_TIMEOUT 0.01 // s
#define TORQUE_MAX_PERCENT 1.00 // 100%
#define STATUS_REQUEST_INTERVAL_MS 50 // 50ms interval for status requests
#define BAMOCAR_CAN_TIMEOUT_MS 10 // Timeout for adding msg to HAL CAN Tx Mailbox
#define N_IGBT 32 // Number of data points in the table

typedef struct {
    uint8_t data[5];
    uint8_t dataLength;
} M_data;

typedef struct {
    int16_t N_ACTUAL;
    int16_t N_ACTUAL_FILT;
    int16_t N_MAX;
    int16_t TORQUE;
    uint16_t READY;
    uint16_t I_DEVICE;
    uint16_t I_200PC;
    uint16_t RAMP_ACC;
    uint16_t RAMP_DEC;
    uint16_t TEMP_MOTOR;
    uint16_t TEMP_IGBT;
    uint16_t TEMP_AIR;
    uint16_t HARD_ENABLED;
    uint16_t V_BUS;
    uint32_t STATUS;
    uint16_t ERROR_STATUS;
    //New Values (7 Sept. 2025)
    //New Current Measurements
    int16_t I_Q_CMD;
    int16_t I_Q_CMD_RAMP;
    int16_t I_ACTUAL;
    int16_t I_Q_ACTUAL;
    int16_t I1_ACTUAL;
    int16_t I2_ACTUAL;
    int16_t I3_ACTUAL;
    int16_t I_ACTUAL_FILT;
    int16_t I_D_ACTUAL;
    int16_t I_Q_ERROR;
    int16_t I_D_ERROR;
    int16_t V_EMF;
    int16_t V_Q;
    int16_t V_D;
    int16_t V_OUTPUT_PHASE;
    uint16_t V_DC_ACTUAL;
    uint16_t BAMOCAR_TORQUE_MAPPED_VALUE;	//experimental

} BamocarData;

typedef struct {
    uint16_t rxID;
    uint16_t txID;
    BamocarData got;
    CAN_TxHeaderTypeDef *TxHeader;
    CAN_RxHeaderTypeDef *RxHeader;
    uint8_t *TxData;
    uint8_t *RxData;
    uint32_t *TxMailbox;
    bool periodicStatusEnabled; // Flag to control periodic status requests
    uint32_t lastStatusRequestTime; // Timestamp of last status request
    CAN_HandleTypeDef *hcan; // Pointer to CAN handle
} Bamocar;

extern Bamocar *g_p_bamocar_debug;

void Bamocar_init(Bamocar *bamocar, uint16_t rxID, uint16_t txID, CAN_TxHeaderTypeDef *TxHeader, CAN_RxHeaderTypeDef *RxHeader, uint8_t *TxData, uint8_t *RxData, uint32_t *TxMailbox , CAN_HandleTypeDef *hcan);
void Bamocar_sendCAN(Bamocar *bamocar, uint8_t dataLength);
void Bamocar_listenCAN(Bamocar *bamocar);
bool Bamocar_parseMessage(Bamocar *bamocar, CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
int16_t Bamocar_getReceived16Bit(uint8_t *data);
int32_t Bamocar_getReceived32Bit(uint8_t *data);

float Bamocar_getSpeed(Bamocar *bamocar);
bool Bamocar_setSpeed(Bamocar *bamocar, int16_t speed);
bool Bamocar_requestSpeed(Bamocar *bamocar, uint8_t interval);

bool Bamocar_setAccel(Bamocar *bamocar, int16_t accel);
bool Bamocar_setDecel(Bamocar *bamocar, int16_t decel);

float Bamocar_getTorque(Bamocar *bamocar);
bool Bamocar_setTorque(Bamocar *bamocar, uint16_t torque);
bool Bamocar_requestTorque(Bamocar *bamocar, uint8_t interval);

float Bamocar_getCurrent(Bamocar *bamocar);
bool Bamocar_requestCurrent(Bamocar *bamocar, uint8_t interval);

uint16_t Bamocar_getMotorTemp(Bamocar *bamocar);
bool Bamocar_requestMotorTemp(Bamocar *bamocar, uint8_t interval);

float Bamocar_getControllerTemp(Bamocar *bamocar);
bool Bamocar_requestControllerTemp(Bamocar *bamocar, uint8_t interval);

uint16_t Bamocar_getAirTemp(Bamocar *bamocar);
bool Bamocar_requestAirTemp(Bamocar *bamocar, uint8_t interval);

uint32_t Bamocar_getStatus(Bamocar *bamocar);
bool Bamocar_requestStatus(Bamocar *bamocar, uint8_t interval);

// Voltage Getters and Request functions
uint16_t Bamocar_getOutputVoltage(Bamocar *bamocar);
bool Bamocar_requestOutputVoltage(Bamocar *bamocar, uint8_t interval);
uint16_t Bamocar_getBusVoltage(Bamocar *bamocar);

float Bamocar_getDCVoltage(Bamocar *bamocar);
bool Bamocar_requestDCVoltage(Bamocar *bamocar, uint8_t interval);

void Bamocar_setSoftEnable(Bamocar *bamocar, bool enable);
bool Bamocar_getHardEnable(Bamocar *bamocar);
bool Bamocar_requestHardEnabled(Bamocar *bamocar, uint8_t interval);

// New functions for periodic status requests
bool Bamocar_requestPeriodicStatus(Bamocar *bamocar, bool enable); // Enable/disable periodic status requests
void Bamocar_periodicStatusTask(Bamocar *bamocar); // Function to be called periodically (e.g., in main loop)
bool Bamocar_checkStatus(Bamocar *bamocar); // Function to check the status and error flags
void Bamocar_SendConvertedDataToCan1(CAN_HandleTypeDef *hcan);

//New
// Subscribe to all FS-needed Bamocar registers at a fixed interval
bool Bamocar_subscribeFSFields(Bamocar *bamocar, uint8_t interval);

#endif/* SRC_BAMOCAR_CAN_H_ */
