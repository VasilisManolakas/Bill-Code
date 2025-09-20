/*
 * bamocar-can.c
 *
 *  Created on: Oct 24, 2024
 *      Author: savva
 */
#include "bamocar-can.h"

extern CAN_HandleTypeDef hcan;

Bamocar *g_p_bamocar_debug = NULL;

double d3_values[N] = { 7414, 7687, 7962, 8240, 8520, 8802, 9085, 9369, 9654,
		9939, 10225, 10510, 10795, 11080, 11364, 11646, 11927, 12207, 12485,
		12762, 13036, 13308, 13578, 13846, 14111, 14373, 14633, 14890, 15144,
		15391, 15628, 15852, 16061, 16251, 16421, 16569, 16693, 16789, 16857 };

double temperatures[N] = { -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20,
		25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105,
		110, 115, 120, 125, 130, 135, 140, 145, 150, 155 };


// Array of raw 16-bit values from the Bamocar for IGBT temp
static const int igbt_raw_values[N_IGBT] = {
    16308, 16387, 16487, 16609, 16757, 16938, 17151, 17400,
    17688, 18017, 18387, 18797, 19247, 19733, 20250, 20793,
    21357, 21933, 22515, 23097, 23671, 24232, 24775, 25296,
    25792, 26261, 26702, 27114, 27497, 27851, 28179, 28480
};

// Corresponding temperature values in degrees Celsius
static const float igbt_temperatures[N_IGBT] = {
    -30.0f, -25.0f, -20.0f, -15.0f, -10.0f, -5.0f,  0.0f,   5.0f,
     10.0f,  15.0f,  20.0f,  25.0f,  30.0f,  35.0f, 40.0f,  45.0f,
     50.0f,  55.0f,  60.0f,  65.0f,  70.0f,  75.0f, 80.0f,  85.0f,
     90.0f,  95.0f, 100.0f, 105.0f, 110.0f, 115.0f, 120.0f, 125.0f
};

void Bamocar_init(Bamocar *bamocar, uint16_t rxID, uint16_t txID,
		CAN_TxHeaderTypeDef *TxHeader, CAN_RxHeaderTypeDef *RxHeader,
		uint8_t *TxData, uint8_t *RxData, uint32_t *TxMailbox,
		CAN_HandleTypeDef *hcan) {

	bamocar->rxID = rxID;
	bamocar->txID = txID;
	bamocar->TxHeader = TxHeader;
	bamocar->RxHeader = RxHeader;
	bamocar->TxData = TxData;
	bamocar->RxData = RxData;
	bamocar->TxMailbox = TxMailbox;
	bamocar->periodicStatusEnabled = false; // Initialize periodic status as disabled
	bamocar->lastStatusRequestTime = 0;
	bamocar->hcan = hcan; // Assign the CAN handle

	memset(&bamocar->got, 0, sizeof(BamocarData));

	g_p_bamocar_debug = bamocar;
}

bool Bamocar_requestData(Bamocar *bamocar, uint8_t dataAddress,
		uint8_t interval) {

	if (bamocar == NULL || bamocar->hcan == NULL) {
		return false;
	}

	bamocar->TxHeader->StdId = bamocar->rxID;
	//kanonika htan 	bamocar->TxHeader->StdId = bamocar->txID;
	bamocar->TxHeader->RTR = CAN_RTR_DATA;
	bamocar->TxHeader->IDE = CAN_ID_STD;
	bamocar->TxHeader->DLC = 0x03;

	bamocar->TxData[0] = REG_REQUEST;
	bamocar->TxData[1] = dataAddress;
	bamocar->TxData[2] = interval;

	if (HAL_CAN_AddTxMessage(bamocar->hcan, bamocar->TxHeader, bamocar->TxData,
			bamocar->TxMailbox) != HAL_OK) {
		return false;
	}

	return true;
}

bool Bamocar_parseMessage(Bamocar *bamocar, CAN_RxHeaderTypeDef *rxHeader,
		uint8_t *rxData) {
	if (rxHeader->StdId == bamocar->txID) {
		int64_t receivedData = 0;
		if (rxHeader->DLC == 4) {
			receivedData = Bamocar_getReceived16Bit(rxData);
		} else if (rxHeader->DLC == 6) {
			receivedData = Bamocar_getReceived32Bit(rxData);
		} else {
			return false;
		}

		switch (rxData[0]) {
				case REG_STATUS:        bamocar->got.STATUS        = (uint32_t)receivedData; break;
		        case REG_READY:         bamocar->got.READY         = (uint16_t)receivedData; break;
		        case REG_ERROR_STATUS:  bamocar->got.ERROR_STATUS  = (uint16_t)receivedData; break;

		        // --- Speed / torque ---
		        case REG_N_ACTUAL:         bamocar->got.N_ACTUAL         = (int16_t)receivedData; break;
		        case REG_N_ACTUAL_FILT:    bamocar->got.N_ACTUAL_FILT    = (int16_t)receivedData; break;
		        case REG_N_MAX:            bamocar->got.N_MAX            = (int16_t)receivedData; break;
		        case REG_TORQUE:           bamocar->got.TORQUE           = (int16_t)receivedData; break;

		        // --- Currents (device & scaling) ---
		        case REG_I_DEVICE:         bamocar->got.I_DEVICE         = (uint16_t)receivedData; break;
		        case REG_I_200PC:          bamocar->got.I_200PC          = (uint16_t)receivedData; break;

		        // --- Currents (new) ---
		        case REG_I_ACTUAL:         bamocar->got.I_ACTUAL         = (int16_t)receivedData; break;
		        case REG_I_Q_CMD:           bamocar->got.I_Q_CMD          = (int16_t)receivedData; break;
		        case REG_IQ_CMD_RAMP:      bamocar->got.I_Q_CMD_RAMP     = (int16_t)receivedData; break;
		        case REG_I_D_ACTUAL:        bamocar->got.I_D_ACTUAL       = (int16_t)receivedData; break;
		        case REG_I_Q_ERROR:         bamocar->got.I_Q_ERROR        = (int16_t)receivedData; break;
		        case REG_I_D_ERROR:         bamocar->got.I_D_ERROR        = (int16_t)receivedData; break;
		        case REG_I1_ACTUAL:        bamocar->got.I1_ACTUAL        = (int16_t)receivedData; break;
		        case REG_I2_ACTUAL:        bamocar->got.I2_ACTUAL        = (int16_t)receivedData; break;
		        case REG_I3_ACTUAL:        bamocar->got.I3_ACTUAL        = (int16_t)receivedData; break;
		        case REG_I_ACTUAL_FILT:    bamocar->got.I_ACTUAL_FILT    = (int16_t)receivedData; break;

		        // --- Voltages ---
		        case REG_V_OUTPUT_PHASE:   bamocar->got.V_OUTPUT_PHASE   = (int16_t)receivedData; break;
		        case REG_V_DC_ACTUAL:      bamocar->got.V_DC_ACTUAL      = (uint16_t)receivedData; break;
		        case REG_V_BUS:            bamocar->got.V_BUS            = (uint16_t)receivedData; break;
		        case REG_I_Q_ACTUAL:     bamocar->got.I_Q_ACTUAL     = (int16_t)receivedData; break;

		        // --- FOC / back-EMF components ---
		        case REG_V_EMF: {  // 0x29; V_EMF (low 16) and V_Q (high 16) share this ID
		            if (rxHeader->DLC == 6) {
		                //already built receivedData little-endian: bytes [1..4]
		                int16_t v_emf = (int16_t)( receivedData        & 0xFFFF);
		                int16_t v_q   = (int16_t)((receivedData >> 16) & 0xFFFF);
		                bamocar->got.V_EMF = v_emf;
		                bamocar->got.V_Q   = v_q;
		            } else { // DLC == 4: only low word present
		                bamocar->got.V_EMF = (int16_t)receivedData;
		                // leave V_Q unchanged (or set to 0 if you prefer)
		            }
		            break;
		        }
		        case REG_V_D:                  bamocar->got.V_D              = (int16_t)receivedData; break;
		        // --- Ramps ---
		        case REG_RAMP_ACC:         bamocar->got.RAMP_ACC         = (uint16_t)receivedData; break;
		        case REG_RAMP_DEC:         bamocar->got.RAMP_DEC         = (uint16_t)receivedData; break;

		        // --- Temperatures / enable ---
		        case REG_TEMP_MOTOR:       bamocar->got.TEMP_MOTOR       = (uint16_t)receivedData; break;
		        case REG_TEMP_IGBT:        bamocar->got.TEMP_IGBT        = (uint16_t)receivedData; break;
		        case REG_TEMP_AIR:         bamocar->got.TEMP_AIR         = (uint16_t)receivedData; break;
		        case REG_HARD_ENABLED:     bamocar->got.HARD_ENABLED     = (uint16_t)receivedData; break;

		        default:
		            return false; // unknown register id
		}

		return true;
	}

	return false;
}

int16_t Bamocar_getReceived16Bit(uint8_t *data) {
	int16_t returnValue;
	returnValue = data[1];
	returnValue |= (data[2] << 8);
	return returnValue;
}

int32_t Bamocar_getReceived32Bit(uint8_t *data) {
	int32_t returnValue;
	returnValue = data[1];
	returnValue |= (data[2] << 8);
	returnValue |= (data[3] << 16);
	returnValue |= (data[4] << 24);
	return returnValue;
}

float Bamocar_getSpeed(Bamocar *bamocar) {
	return bamocar->got.N_MAX * (-bamocar->got.N_ACTUAL / 32767.0);
}

bool Bamocar_setDecel(Bamocar *bamocar, int16_t decel) {
	uint8_t TxData[3] = { REG_RAMP_DEC, decel & 0xFF, (decel >> 8) & 0xFF };

	bamocar->TxHeader->StdId = bamocar->rxID;
	bamocar->TxHeader->RTR = CAN_RTR_DATA;
	bamocar->TxHeader->IDE = CAN_ID_STD;
	bamocar->TxHeader->DLC = 3;

	if (HAL_CAN_AddTxMessage(bamocar->hcan, bamocar->TxHeader, TxData,
			bamocar->TxMailbox) != HAL_OK) {

		return false;
	}
	return true;
}

bool Bamocar_requestSpeed(Bamocar *bamocar, uint8_t interval) {
	bool success = true;
	if (!Bamocar_requestData(bamocar, REG_N_ACTUAL, interval))
		success = false;
	if (!Bamocar_requestData(bamocar, REG_N_MAX, interval))
		success = false;
	return success;
}

bool Bamocar_setAccel(Bamocar *bamocar, int16_t accel) {
	bamocar->TxData[0] = REG_RAMP_ACC;
	bamocar->TxData[1] = accel & 0xFF;
	bamocar->TxData[2] = (accel >> 8) & 0xFF;
	Bamocar_sendCAN(bamocar, 3);
	return true;
}

float Bamocar_getTorque(Bamocar *bamocar) {
	return bamocar->got.TORQUE / 0x5555;
}

bool Bamocar_setTorque(Bamocar *bamocar, uint16_t torque) {

	uint8_t TxData[3] = { REG_TORQUE, torque & 0xFF, (torque >> 8) & 0xFF };

	bamocar->TxHeader->StdId = bamocar->rxID;
	bamocar->TxHeader->RTR = CAN_RTR_DATA;
	bamocar->TxHeader->IDE = CAN_ID_STD;
	bamocar->TxHeader->DLC = 3;

	if (HAL_CAN_AddTxMessage(bamocar->hcan, bamocar->TxHeader, TxData,
			bamocar->TxMailbox) != HAL_OK) {

		return false;
	}
	return true;
}

bool Bamocar_requestTorque(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_TORQUE, interval);
}

float Bamocar_getCurrent(Bamocar *bamocar) {
	return ((2.0 / 10.0) * bamocar->got.I_DEVICE
			* (bamocar->got.I_ACTUAL / (float) bamocar->got.I_200PC));
}

bool Bamocar_requestCurrent(Bamocar *bamocar, uint8_t interval) {
	bool success = true;
	if (!Bamocar_requestData(bamocar, REG_I_ACTUAL, interval))
		success = false;
	if (!Bamocar_requestData(bamocar, REG_I_DEVICE, interval))
		success = false;
	if (!Bamocar_requestData(bamocar, REG_I_200PC, interval))
		success = false;
	return success;
}

double interpolate(double x) {
	// Handle edge cases first
	if (x <= d3_values[0])
		return temperatures[0];
	if (x >= d3_values[N - 1])
		return temperatures[N - 1];

	// Use binary search instead of linear search for better performance
	int low = 0;
	int high = N - 1;

	while (low <= high) {
		int mid = (low + high) / 2;

		if (x >= d3_values[mid] && x <= d3_values[mid + 1]) {
			// Found the interval, now interpolate
			double t = (x - d3_values[mid])
					/ (d3_values[mid + 1] - d3_values[mid]);
			return temperatures[mid]
					+ t * (temperatures[mid + 1] - temperatures[mid]);
		}

		if (x < d3_values[mid])
			high = mid - 1;
		else
			low = mid + 1;
	}

	// This should never be reached given the edge case checks
	return 0;
}

static float interpolate_igbt_temp(int raw_value) {
    // Handle edge cases where the value is outside the table's range
    if (raw_value <= igbt_raw_values[0]) {
        return igbt_temperatures[0];
    }
    if (raw_value >= igbt_raw_values[N_IGBT - 1]) {
        return igbt_temperatures[N_IGBT - 1];
    }

    // Find the interval in which the raw_value lies using a linear search.
    // (A binary search would be faster, but for 32 elements, linear is perfectly fine and simple).
    for (int i = 0; i < N_IGBT - 1; i++) {
        if (raw_value >= igbt_raw_values[i] && raw_value <= igbt_raw_values[i + 1]) {
            // We found the two points to interpolate between.
            // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            float x1 = (float)igbt_raw_values[i];
            float x2 = (float)igbt_raw_values[i + 1];
            float y1 = igbt_temperatures[i];
            float y2 = igbt_temperatures[i + 1];

            return y1 + ((float)raw_value - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    // This part should not be reached if the edge cases are handled, but as a fallback,
    // return the maximum temperature.
    return igbt_temperatures[N_IGBT - 1];
}

uint16_t Bamocar_getMotorTemp(Bamocar *bamocar) {
	return interpolate(bamocar->got.TEMP_MOTOR);
}

bool Bamocar_requestMotorTemp(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_TEMP_MOTOR, interval);
}

float Bamocar_getControllerTemp(Bamocar *bamocar) {
    // Convert the raw value to a real temperature using our new interpolation function
    return interpolate_igbt_temp(bamocar->got.TEMP_IGBT);
}

bool Bamocar_requestControllerTemp(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_TEMP_IGBT, interval);
}

uint16_t Bamocar_getAirTemp(Bamocar *bamocar) {
	return bamocar->got.TEMP_AIR;
}

bool Bamocar_requestAirTemp(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_TEMP_AIR, interval);
}

uint32_t Bamocar_getStatus(Bamocar *bamocar) {
	return bamocar->got.STATUS;
}

bool Bamocar_requestStatus(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_STATUS, interval);
}

void Bamocar_setSoftEnable(Bamocar *bamocar, bool enable) {
	uint8_t m_data2 = enable ? 0x00 : 0x04;
	uint8_t TxData[3] = { REG_ENABLE, m_data2, 0x00 };

	bamocar->TxHeader->StdId = bamocar->rxID;
	bamocar->TxHeader->RTR = CAN_RTR_DATA;
	bamocar->TxHeader->IDE = CAN_ID_STD;
	bamocar->TxHeader->DLC = 3;

	if (HAL_CAN_AddTxMessage(bamocar->hcan, bamocar->TxHeader, TxData,
			bamocar->TxMailbox) != HAL_OK) {

	}
}

bool Bamocar_getHardEnable(Bamocar *bamocar) {
	return bamocar->got.HARD_ENABLED;
}

bool Bamocar_requestHardEnabled(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_HARD_ENABLED, interval);
}



uint16_t Bamocar_getBusVoltage(Bamocar *bamocar) {
	return (uint16_t) (bamocar->got.V_BUS) / 31.5f;
}

uint16_t Bamocar_getOutputVoltage(Bamocar *bamocar) {
	return bamocar->got.V_OUTPUT_PHASE;
}

bool Bamocar_requestOutputVoltage(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_V_OUTPUT_PHASE, interval);
}

float Bamocar_getDCVoltage(Bamocar *bamocar) {
	// The received value is a raw integer. We scale it to get the actual voltage.
	return (float) bamocar->got.V_DC_ACTUAL;
}

bool Bamocar_requestDCVoltage(Bamocar *bamocar, uint8_t interval) {
	return Bamocar_requestData(bamocar, REG_V_DC_ACTUAL, interval);
}

// Periodic Status Request Functions
bool Bamocar_requestPeriodicStatus(Bamocar *bamocar, bool enable) {
	bamocar->periodicStatusEnabled = enable;
	return true; // Return true, assuming enabling/disabling always succeeds
}

void Bamocar_periodicStatusTask(Bamocar *bamocar) {
	if (!bamocar->periodicStatusEnabled)
		return; // Exit if not enabled

	uint32_t currentTime = HAL_GetTick();
	if (currentTime
			- bamocar->lastStatusRequestTime>= STATUS_REQUEST_INTERVAL_MS) {
		bamocar->lastStatusRequestTime = currentTime;
		Bamocar_requestData(bamocar, REG_STATUS, INTVL_IMMEDIATE); // Request Status
		Bamocar_requestData(bamocar, REG_ERROR_STATUS, INTVL_IMMEDIATE); // Request Error Status
	}
}

bool Bamocar_checkStatus(Bamocar *bamocar) {
	// Add your status checking logic here based on bamocar->got.STATUS and bamocar->got.ERROR_STATUS
	// Example status check (modify as per Bamocar D3 documentation):
	if (bamocar->got.STATUS & (1 << 0)) { // Example: Check bit 0 for some status
		printf("Status Bit 0 is set!\n");
		return false; // Or handle the status as needed
	}
	if (bamocar->got.ERROR_STATUS != 0) { // Example: Check if Error Status is not zero
		printf("Error Status is NOT 0: %d\n", bamocar->got.ERROR_STATUS);
		return false; // Indicate error condition
	}
	return true; // No error/status conditions detected (based on your checks)
}

// Packages and transmits all data from BamocarData
// Create new CAN messages based on the BamocarData structure
// give id number above 0x185

void Bamocar_SendConvertedDataToCan1(CAN_HandleTypeDef *hcan)
{
    // Extra CAN IDs (kept local to this function to avoid touching headers)
    const uint16_t ID_CURRENTS_A = 0x188; // I_Q_CMD, I_Q_CMD_RAMP, I_ACTUAL, I_Q_ACTUAL
    const uint16_t ID_CURRENTS_B = 0x189; // I1_ACTUAL, I2_ACTUAL, I3_ACTUAL, I_ACTUAL_FILT
    const uint16_t ID_FOC_VOLT   = 0x18A; // I_D_ACTUAL, I_Q_ERROR, I_D_ERROR, N_ACTUAL_FILT
    const uint16_t ID_VOLTAGES   = 0x18B; // V_D, V_EMF, V_Q, V_DC_ACTUAL

    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8];
    uint32_t            tx_mailbox;

    // Must have a valid global pointer set during Bamocar_init()
    if (g_p_bamocar_debug == NULL) {
        return;
    }

    // Common header fields
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;

    // ------------------------------------------------------------------------
    // Message 1: Operational Data (kept as in your code)  ID: 0x186, DLC: 6
    // speed_rpm: int16 (from Bamocar_getSpeed)
    // torque_scaled: int16 (torque [-1..1] * 10000)
    // dc_voltage: uint16 (scaled bus voltage via Bamocar_getBusVoltage)
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = BAMOCAR_TX_ID_OPERATIONAL_DATA;
        tx_header.DLC   = 6;

        int16_t  speed_rpm      = (int16_t)Bamocar_getSpeed(g_p_bamocar_debug);
        int16_t  torque_scaled  = (int16_t)(Bamocar_getTorque(g_p_bamocar_debug) * 10000.0f);
        uint16_t dc_voltage     = Bamocar_getBusVoltage(g_p_bamocar_debug);

        tx_data[0] = (uint8_t)(speed_rpm & 0xFF);
        tx_data[1] = (uint8_t)(speed_rpm >> 8);
        tx_data[2] = (uint8_t)(torque_scaled & 0xFF);
        tx_data[3] = (uint8_t)(torque_scaled >> 8);
        tx_data[4] = (uint8_t)(dc_voltage & 0xFF);
        tx_data[5] = (uint8_t)(dc_voltage >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }

    // ------------------------------------------------------------------------
    // Message 2: Temperatures (kept as in your code)  ID: 0x187, DLC: 4
    // motor_temp_scaled: int16 (°C * 100)
    // controller_temp_scaled: int16 (°C * 100)
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = BAMOCAR_TX_ID_TEMPERATURE_DATA;
        tx_header.DLC   = 4;

        int16_t motor_temp_scaled     = (int16_t)(Bamocar_getMotorTemp(g_p_bamocar_debug) * 100.0f);
        int16_t controller_temp_scaled= (int16_t)(Bamocar_getControllerTemp(g_p_bamocar_debug) * 100.0f);

        tx_data[0] = (uint8_t)(motor_temp_scaled & 0xFF);
        tx_data[1] = (uint8_t)(motor_temp_scaled >> 8);
        tx_data[2] = (uint8_t)(controller_temp_scaled & 0xFF);
        tx_data[3] = (uint8_t)(controller_temp_scaled >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }

    // ------------------------------------------------------------------------
    // Message 3: Currents A  ID: 0x188, DLC: 8
    // I_Q_CMD, I_Q_CMD_RAMP, I_ACTUAL, I_Q_ACTUAL (all int16)
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = ID_CURRENTS_A;
        tx_header.DLC   = 8;

        int16_t iq_cmd       = g_p_bamocar_debug->got.I_Q_CMD;
        int16_t iq_cmd_ramp  = g_p_bamocar_debug->got.I_Q_CMD_RAMP;
        int16_t i_actual     = g_p_bamocar_debug->got.I_ACTUAL;
        int16_t iq_actual    = g_p_bamocar_debug->got.I_Q_ACTUAL;

        tx_data[0] = (uint8_t)(iq_cmd & 0xFF);
        tx_data[1] = (uint8_t)(iq_cmd >> 8);
        tx_data[2] = (uint8_t)(iq_cmd_ramp & 0xFF);
        tx_data[3] = (uint8_t)(iq_cmd_ramp >> 8);
        tx_data[4] = (uint8_t)(i_actual & 0xFF);
        tx_data[5] = (uint8_t)(i_actual >> 8);
        tx_data[6] = (uint8_t)(iq_actual & 0xFF);
        tx_data[7] = (uint8_t)(iq_actual >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }

    // ------------------------------------------------------------------------
    // Message 4: Currents B  ID: 0x189, DLC: 8
    // I1_ACTUAL, I2_ACTUAL, I3_ACTUAL, I_ACTUAL_FILT (all int16)
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = ID_CURRENTS_B;
        tx_header.DLC   = 8;

        int16_t i1 = g_p_bamocar_debug->got.I1_ACTUAL;
        int16_t i2 = g_p_bamocar_debug->got.I2_ACTUAL;
        int16_t i3 = g_p_bamocar_debug->got.I3_ACTUAL;
        int16_t i_filtered = g_p_bamocar_debug->got.I_ACTUAL_FILT;

        tx_data[0] = (uint8_t)(i1 & 0xFF);
        tx_data[1] = (uint8_t)(i1 >> 8);
        tx_data[2] = (uint8_t)(i2 & 0xFF);
        tx_data[3] = (uint8_t)(i2 >> 8);
        tx_data[4] = (uint8_t)(i3 & 0xFF);
        tx_data[5] = (uint8_t)(i3 >> 8);
        tx_data[6] = (uint8_t)(i_filtered & 0xFF);
        tx_data[7] = (uint8_t)(i_filtered >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }

    // ------------------------------------------------------------------------
    // Message 5: FOC + filtered speed   ID: 0x18A, DLC: 8
    // I_D_ACTUAL, I_Q_ERROR, I_D_ERROR, N_ACTUAL_FILT (all int16)
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = ID_FOC_VOLT;
        tx_header.DLC   = 8;

        int16_t id_actual    = g_p_bamocar_debug->got.I_D_ACTUAL;
        int16_t iq_error     = g_p_bamocar_debug->got.I_Q_ERROR;
        int16_t id_error     = g_p_bamocar_debug->got.I_D_ERROR;
        int16_t n_filt       = g_p_bamocar_debug->got.N_ACTUAL_FILT;

        tx_data[0] = (uint8_t)(id_actual & 0xFF);
        tx_data[1] = (uint8_t)(id_actual >> 8);
        tx_data[2] = (uint8_t)(iq_error & 0xFF);
        tx_data[3] = (uint8_t)(iq_error >> 8);
        tx_data[4] = (uint8_t)(id_error & 0xFF);
        tx_data[5] = (uint8_t)(id_error >> 8);
        tx_data[6] = (uint8_t)(n_filt & 0xFF);
        tx_data[7] = (uint8_t)(n_filt >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }

    // ------------------------------------------------------------------------
    // Message 6: Voltages  ID: 0x18B, DLC: 8
    // V_D, V_EMF, V_Q, V_DC_ACTUAL (all 16-bit; V_DC_ACTUAL is uint16)
    // Note: V_EMF / V_Q are already split into int16 values in the parser.
    // ------------------------------------------------------------------------
    {
        memset(tx_data, 0, sizeof(tx_data));
        tx_header.StdId = ID_VOLTAGES;
        tx_header.DLC   = 8;

        int16_t  v_d   = g_p_bamocar_debug->got.V_D;
        int16_t  v_emf = g_p_bamocar_debug->got.V_EMF;
        int16_t  v_q   = g_p_bamocar_debug->got.V_Q;
        uint16_t v_dc  = g_p_bamocar_debug->got.V_DC_ACTUAL;

        tx_data[0] = (uint8_t)(v_d & 0xFF);
        tx_data[1] = (uint8_t)(v_d >> 8);
        tx_data[2] = (uint8_t)(v_emf & 0xFF);
        tx_data[3] = (uint8_t)(v_emf >> 8);
        tx_data[4] = (uint8_t)(v_q & 0xFF);
        tx_data[5] = (uint8_t)(v_q >> 8);
        tx_data[6] = (uint8_t)(v_dc & 0xFF);
        tx_data[7] = (uint8_t)(v_dc >> 8);

        HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
    }
}

//New
bool Bamocar_subscribeFSFields(Bamocar *bamocar, uint8_t interval)
{
    bool ok = true;

    // --- Currents
    ok &= Bamocar_requestData(bamocar, REG_I_Q_CMD,        interval);
    ok &= Bamocar_requestData(bamocar, REG_IQ_CMD_RAMP,    interval);
    ok &= Bamocar_requestData(bamocar, REG_I_ACTUAL,       interval);
    ok &= Bamocar_requestData(bamocar, REG_I_Q_ACTUAL,     interval);
    ok &= Bamocar_requestData(bamocar, REG_I1_ACTUAL,      interval);
    ok &= Bamocar_requestData(bamocar, REG_I2_ACTUAL,      interval);
    ok &= Bamocar_requestData(bamocar, REG_I3_ACTUAL,      interval);
    ok &= Bamocar_requestData(bamocar, REG_I_ACTUAL_FILT,  interval);
    ok &= Bamocar_requestData(bamocar, REG_I_D_ACTUAL,     interval);
    ok &= Bamocar_requestData(bamocar, REG_I_Q_ERROR,      interval);
    ok &= Bamocar_requestData(bamocar, REG_I_D_ERROR,      interval);

    // --- Voltages
    // REG_V_EMF (0x29) may return V_EMF(low16) and V_Q(high16) when DLC==6 — your parser already handles this.
    ok &= Bamocar_requestData(bamocar, REG_V_EMF,          interval);
    ok &= Bamocar_requestData(bamocar, REG_V_D,            interval);
    ok &= Bamocar_requestData(bamocar, REG_V_OUTPUT_PHASE, interval);
    ok &= Bamocar_requestData(bamocar, REG_V_DC_ACTUAL,    interval);

    // --- Filtered RPM (you asked for N_ACTUAL_FILT) ---
    ok &= Bamocar_requestData(bamocar, REG_N_ACTUAL_FILT,  interval);

    return ok;
}


/* experimental
 *
 * int16_t I_Q_CMD;
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

 */

