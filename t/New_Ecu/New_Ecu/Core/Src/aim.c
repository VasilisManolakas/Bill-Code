/*
 * aim.c
 *
 *  Created on: Apr 14, 2025
 *      Author: savva
 */

#include "aim.h"
#include "main.h"      // For peripheral handles (ADC, SPI, UART, TIM, EXTI)
#include <string.h>    // For memset
#include <stdio.h>     // For printf debugging
#include <stdlib.h>    // For rand() if simulating
#include <math.h>		// For sin/cos if simulating or processing sensor data
#include "aim-registers.h"     // AiM_CAN_ID_* and function prototypes
#include "shared_spi_data.h"
extern Bamocar myBamocar;  // defined in main.c after Bamocar_init(...)

// --- Private Global Data Variable ---
static AIM_Data_t g_aim_data;
#include <math.h>   // for lroundf

static inline int16_t clamp_i16(int32_t v) {
	if (v < -32768)
		return -32768;
	if (v > 32767)
		return 32767;
	return (int16_t) v;
}
static inline uint16_t clamp_u16(int32_t v) {
	if (v < 0)
		return 0;
	if (v > 65535)
		return 65535;
	return (uint16_t) v;
}

// --- Initialization ---
bool AIM_Init(void) {
	// --- TODO: Initialize interfaces for ALL AIM sensors/sources ---
	// Example: Initialize SPI for an IMU (e.g., MPU6050/ICM-20948)
	// MX_SPIx_Init(); // Assuming CubeMX generated init
	// Configure IMU registers via SPI write commands...
	// printf("AIM IMU Sensor Initialized.\n");

	// Example: Initialize UART for GPS (e.g., using HAL IT or DMA)
	// MX_USARTx_UART_Init();
	// Start receiving GPS data:
	// HAL_UART_Receive_DMA(&huart_gps, gps_rx_buffer, GPS_BUFFER_SIZE);
	// Or: HAL_UART_Receive_IT(&huart_gps, &gps_rx_char, 1); // Receive one char at a time
	// printf("AIM GPS UART Initialized.\n");

	// Example: Initialize ADC for Analog Sensors (Pedal, Brake, Steering)
	// MX_ADCx_Init();
	// HAL_ADC_Start_DMA(&hadc_sensors, (uint32_t*)adc_values, NUM_ADC_CHANNELS); // Start continuous conversion
	// printf("AIM ADC Initialized.\n");

	// Example: Initialize Timers in Input Capture or EXTI for Wheel Speeds
	// MX_TIMx_Init(); // Configure Timer in Input Capture mode
	// HAL_TIM_IC_Start_IT(&htim_wheel_rb, TIM_CHANNEL_1); // Start capture for one wheel
	// Or configure GPIOs as EXTI lines for rising/falling edges from sensors
	// printf("AIM Wheel Speed Timers/EXTI Initialized.\n");

	// Initialize data structure
	memset(&g_aim_data, 0, sizeof(AIM_Data_t));
	g_aim_data.gps_latitude = 40.7128f; // Default location example
	g_aim_data.gps_longitude = -74.0060f;

	printf("AIM Module Initialized.\n");
	return true;
}

// --- Data Update ---
void AIM_UpdateData(void) {
	// Called periodically from main loop. Read sensors, check flags.

	// --- TODO: Replace with actual data acquisition ---

	// Example: Read IMU data via SPI
	// uint8_t imu_data[14];
	// Read_IMU_Registers(imu_data, 14); // Your function to read sensor data
	// int16_t accel_x = (imu_data[0] << 8) | imu_data[1];
	// g_aim_data.g_sensor_x = (float)accel_x * G_SCALE_FACTOR; // Apply scaling
	// ... parse gyro, temp etc ...
	// g_aim_data.pitch_angle = CalculatePitch(accel_x, accel_y, accel_z); // Sensor fusion/calculation

	// Example: Read ADC values (assuming DMA put them in adc_values array)
	// g_aim_data.pedal_position = (float)adc_values[PEDAL_ADC_CHANNEL] / 4095.0f * 100.0f; // Scale to percentage
	// g_aim_data.brake_position = (float)adc_values[BRAKE_ADC_CHANNEL] * BRAKE_SENSOR_SCALE; // Apply calibration
	// g_aim_data.steering_angle = Convert_ADC_To_Steering(adc_values[STEERING_ADC_CHANNEL]);

	// Example: Check GPS data flag (set by UART DMA/IT callback)
	// extern volatile bool gps_data_ready_flag; // Declared elsewhere
	// if (gps_data_ready_flag) {
	//     Parse_GPS_NMEA_Sentence(gps_rx_buffer, &g_aim_data); // Your NMEA parsing function
	//     gps_data_ready_flag = false;
	//     // Re-arm DMA/IT reception if needed
	//     // HAL_UART_Receive_DMA(&huart_gps, gps_rx_buffer, GPS_BUFFER_SIZE);
	// }

	// Example: Calculate Wheel Speeds (from Timer Input Capture or EXTI counters)
	// uint32_t rb_period = Get_Wheel_Timer_Period(TIMER_RB); // Function to get period/count
	// if (rb_period > 0) {
	//    g_aim_data.wheel_speed_rb = WHEEL_CIRCUMFERENCE * (TIMER_CLOCK_FREQ / rb_period); // Calculate speed
	// } else {
	//    g_aim_data.wheel_speed_rb = 0.0f; // Wheel stopped
	// }
	// Calculate for other wheels...

//    #ifdef AIM_SIMULATE_DATA
//    // Simulate some data if not reading real sensors
//    static float sim_heading = 0.0f;
//    sim_heading += 0.1f; if (sim_heading >= 360.0f) sim_heading -= 360.0f;
//    float angle_rad = sim_heading * 3.14159f / 180.0f;
//    g_aim_data.g_sensor_x = 0.1f * sinf(angle_rad) + ((float)(rand() % 100)/500.0f - 0.1f); // Simulate centripetal + noise
//    g_aim_data.g_sensor_y = 0.1f * cosf(angle_rad) + ((float)(rand() % 100)/500.0f - 0.1f);
//    g_aim_data.g_sensor_z = -1.0f + ((float)(rand() % 100)/1000.0f - 0.05f); // Gravity + noise
//    g_aim_data.pitch_angle = 1.0f + ((float)(rand() % 100)/200.0f - 0.25f);
//    g_aim_data.yaw_moment = 0.5f * sinf(angle_rad * 2.0f); // Simulate yaw based on turn
//    g_aim_data.gps_signal_valid = 1;
//    g_aim_data.gps_latitude += 0.00001f * cosf(angle_rad);
//    g_aim_data.gps_longitude += 0.000015f * sinf(angle_rad);
//    g_aim_data.gps_speed = 15.0f + ((float)(rand() % 100)/50.0f - 1.0f); // km/h? m/s? Define units!
//    g_aim_data.steering_angle = 45.0f * sinf(angle_rad);
//    g_aim_data.pedal_position = 50.0f + 20.0f * cosf(angle_rad * 0.5f);
//    g_aim_data.wheel_speed_rb = g_aim_data.gps_speed * (1.0f + 0.1f * sinf(angle_rad)); // Simulate differential speed
//    g_aim_data.wheel_speed_lb = g_aim_data.gps_speed * (1.0f - 0.1f * sinf(angle_rad));
//    g_aim_data.wheel_speed_rf = g_aim_data.gps_speed * (1.0f + 0.1f * sinf(angle_rad));
//    g_aim_data.wheel_speed_lf = g_aim_data.gps_speed * (1.0f - 0.1f * sinf(angle_rad));
//    g_aim_data.distance += (g_aim_data.gps_speed * (float)AIM_UPDATE_INTERVAL_MS / 3600.0f); // Example distance accumulation
//    #endif // AIM_SIMULATE_DATA
//
//
//    g_aim_data.time = (float)HAL_GetTick() / 1000.0f; // Update system time (seconds)
//    g_aim_data.last_update_time = HAL_GetTick();
}

// --- Message Processing ---
// Call this function from your central CAN Rx Callback if AIM data comes via CAN
void AIM_ProcessMessage(uint8_t *rxData, uint8_t dlc, uint32_t msgId) {
	extern AIM_Data_t aim_data;

	// Safety check
	if (rxData == NULL || dlc == 0) {
		return;
	}

	// Using msgId to determine message type from AIM (0x2)
	// Different IDs or command bytes for different data types
	switch (msgId) {
	case 0x2:  // Main AIM data
		// G-sensor data
		aim_data.g_sensor_x = (float) ((int16_t) (rxData[0] | (rxData[1] << 8)))
				/ 100.0f;
		aim_data.g_sensor_y = (float) ((int16_t) (rxData[2] | (rxData[3] << 8)))
				/ 100.0f;
		aim_data.g_sensor_z = (float) ((int16_t) (rxData[4] | (rxData[5] << 8)))
				/ 100.0f;
		break;

	case 0x3:  // GPS data
		aim_data.gps_signal_valid = rxData[0] & 0x01;
		// GPS coordinates require multiple bytes - using a simplified representation
		aim_data.gps_latitude = (float) ((int32_t) (rxData[1] | (rxData[2] << 8)
				| (rxData[3] << 16) | (rxData[4] << 24))) / 1000000.0f;
		aim_data.gps_longitude = (float) ((int32_t) (rxData[5]
				| (rxData[6] << 8) | (rxData[7] << 16))) / 1000000.0f;
		break;

	case 0x4:  // Speed & motion data
		aim_data.gps_speed = (float) ((uint16_t) (rxData[0] | (rxData[1] << 8)))
				/ 10.0f;
		aim_data.pitch_angle =
				(float) ((int16_t) (rxData[2] | (rxData[3] << 8))) / 100.0f;
		aim_data.yaw_moment = (float) ((int16_t) (rxData[4] | (rxData[5] << 8)))
				/ 100.0f;
		aim_data.angleroll = (float) ((int16_t) (rxData[6] | (rxData[7] << 8)))
				/ 100.0f;
		break;

	case 0x5:  // Wheel speed data
		aim_data.wheel_speed_rb = (float) ((uint16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		aim_data.wheel_speed_lb = (float) ((uint16_t) (rxData[2]
				| (rxData[3] << 8))) / 10.0f;
		aim_data.wheel_speed_rf = (float) ((uint16_t) (rxData[4]
				| (rxData[5] << 8))) / 10.0f;
		aim_data.wheel_speed_lf = (float) ((uint16_t) (rxData[6]
				| (rxData[7] << 8))) / 10.0f;
		break;

	case 0x6:  // Position data
		aim_data.pos_left_back_sensor = (float) ((uint16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		aim_data.pos_right_back_sensor = (float) ((uint16_t) (rxData[2]
				| (rxData[3] << 8))) / 10.0f;
		aim_data.pos_left_front_sensor = (float) ((uint16_t) (rxData[4]
				| (rxData[5] << 8))) / 10.0f;
		aim_data.pos_right_front_sensor = (float) ((uint16_t) (rxData[6]
				| (rxData[7] << 8))) / 10.0f;
		break;

	case 0x7:  // Brake data and pedal position
		aim_data.pedal_position = (float) ((uint16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		aim_data.brake_pressure = (float) ((uint16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		aim_data.brake_position = (float) ((uint16_t) (rxData[2]
				| (rxData[3] << 8))) / 10.0f;
		break;

	case 0x8:  // Steering data
		aim_data.steering_angle = (float) ((int16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		break;
	case 0x9:  // Distance data
		aim_data.distance = (float) ((uint32_t) (rxData[0] | (rxData[1] << 8)
				| (rxData[2] << 16) | (rxData[3] << 24))) / 100.0f;
		aim_data.distance_lap = (float) ((uint32_t) (rxData[4]
				| (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24)))
				/ 100.0f;
		break;

	case 0xA:  // Duration data
		aim_data.duration = (float) ((uint32_t) (rxData[0] | (rxData[1] << 8)
				| (rxData[2] << 16) | (rxData[3] << 24))) / 1000.0f;
		break;

	case 0xB:  // Time data
		aim_data.time = (float) ((uint32_t) (rxData[0] | (rxData[1] << 8)
				| (rxData[2] << 16) | (rxData[3] << 24))) / 1000.0f;
		break;

	case 0xC:  // Speed velocity data
		aim_data.speed_velocity_rb = (float) ((uint16_t) (rxData[0]
				| (rxData[1] << 8))) / 10.0f;
		aim_data.speed_velocity_lb = (float) ((uint16_t) (rxData[2]
				| (rxData[3] << 8))) / 10.0f;
		aim_data.speed_velocity_rf = (float) ((uint16_t) (rxData[4]
				| (rxData[5] << 8))) / 10.0f;
		aim_data.speed_velocity_lf = (float) ((uint16_t) (rxData[6]
				| (rxData[7] << 8))) / 10.0f;
		break;

	default:
		// Unknown message ID
		break;
	}
}

// --- Example GPS Character Processor ---
// Call this from HAL_UART_RxCpltCallback when using single-char IT reception
/*
 #define GPS_LINE_BUFFER_SIZE 100
 static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
 static uint8_t gps_line_index = 0;

 void AIM_ProcessGpsChar(uint8_t rxChar) {
 if (rxChar == '\n') { // End of line detected
 gps_line_buffer[gps_line_index] = '\0'; // Null terminate
 // --- TODO: Parse the NMEA sentence in gps_line_buffer ---
 // Example: Look for $GPRMC or $GPGGA
 // if (strncmp(gps_line_buffer, "$GPRMC", 6) == 0) {
 //    Parse_GPRMC(gps_line_buffer, &g_aim_data); // Your parsing function
 // } else if (strncmp(gps_line_buffer, "$GPGGA", 6) == 0) {
 //    Parse_GPGGA(gps_line_buffer, &g_aim_data); // Your parsing function
 // }
 gps_line_index = 0; // Reset buffer index
 } else if (rxChar != '\r' && gps_line_index < GPS_LINE_BUFFER_SIZE - 1) {
 gps_line_buffer[gps_line_index++] = rxChar; // Add char to buffer
 } else if (gps_line_index >= GPS_LINE_BUFFER_SIZE - 1) {
 // Buffer overflow, discard line
 gps_line_index = 0;
 }
 // Re-arm single character reception in the UART callback itself
 }
 */

// --- Getter Functions ---
float AIM_GetGSensorX(void) {
	return g_aim_data.g_sensor_x;
}
float AIM_GetGSensorY(void) {
	return g_aim_data.g_sensor_y;
}
float AIM_GetGSensorZ(void) {
	return g_aim_data.g_sensor_z;
}
float AIM_GetPitchAngle(void) {
	return g_aim_data.pitch_angle;
}
float AIM_GetYawMoment(void) {
	return g_aim_data.yaw_moment;
}
uint8_t AIM_GetGpsValid(void) {
	return g_aim_data.gps_signal_valid;
}
float AIM_GetGpsLatitude(void) {
	return g_aim_data.gps_latitude;
}
float AIM_GetGpsLongitude(void) {
	return g_aim_data.gps_longitude;
}
float AIM_GetGpsSpeed(void) {
	return g_aim_data.gps_speed;
}
float AIM_GetSteeringAngle(void) {
	return g_aim_data.steering_angle;
}
float AIM_GetBrakePressure(void) {
	return g_aim_data.brake_pressure;
}
float AIM_GetAngleroll(void) {
	return g_aim_data.angleroll;
}
float AIM_GetDistance(void) {
	return g_aim_data.distance;
}
float AIM_GetDistanceLap(void) {
	return g_aim_data.distance_lap;
}
float AIM_GetDuration(void) {
	return g_aim_data.duration;
}
float AIM_GetPosLeftBack(void) {
	return g_aim_data.pos_left_back_sensor;
}
float AIM_GetPosRightBack(void) {
	return g_aim_data.pos_right_back_sensor;
}
float AIM_GetPosLeftFront(void) {
	return g_aim_data.pos_left_front_sensor;
}
float AIM_GetPosRightFront(void) {
	return g_aim_data.pos_right_front_sensor;
}
float AIM_GetPedalPosition(void) {
	return g_aim_data.pedal_position;
}
float AIM_GetBrakePosition(void) {
	return g_aim_data.brake_position;
}
float AIM_GetWheelSpeedRB(void) {
	return g_aim_data.wheel_speed_rb;
}
float AIM_GetWheelSpeedLB(void) {
	return g_aim_data.wheel_speed_lb;
}
float AIM_GetWheelSpeedRF(void) {
	return g_aim_data.wheel_speed_rf;
}
float AIM_GetWheelSpeedLF(void) {
	return g_aim_data.wheel_speed_lf;
}
float AIM_GetSpeedVelocityRB(void) {
	return g_aim_data.speed_velocity_rb;
}
float AIM_GetSpeedVelocityLB(void) {
	return g_aim_data.speed_velocity_lb;
}
float AIM_GetSpeedVelocityRF(void) {
	return g_aim_data.speed_velocity_rf;
}
float AIM_GetSpeedVelocityLF(void) {
	return g_aim_data.speed_velocity_lf;
}
float AIM_GetTime(void) {
	return g_aim_data.time;
}

static CAN_HandleTypeDef *AiM_bus_handle = NULL;	//Unused Pointer as of Now
static CAN_TxHeaderTypeDef log_performance_tx, log_temperatures_tx,
		log_flags_tx; //Unused as of Now
static CAN_TxHeaderTypeDef log_bamo_currA_tx, log_bamo_currB_tx,
		log_bamo_foc_tx, log_bamo_volt_tx;
static CAN_TxHeaderTypeDef log_apps_tx,log_mapped_torque_value_tx;

void LoggerCAN_Send_Performance_Data(const CombinedLogData_t *d)
{
	if (!AiM_bus_handle || ! d)
	        {
	            fprintf(stderr,"%s","AiM bus handle or func param uninitialized");
	            return;
	        }
	        uint8_t CAN_data[8] = {0};
	        uint32_t message_box;
	        int16_t rpm =       clamp_i16((int32_t)lroundf(d->bamocar_rpm));
	        int16_t torque  = clamp_i16((int32_t)lroundf(d->bamocar_torque_actual * 10000.0f)); //Multiply by 10k to convert to integer.
	        uint16_t vdc    = clamp_u16((int32_t)lroundf((float)d->bamocar_dc_voltage_raw / 31.5f));
	        int16_t current = clamp_i16((int32_t)lroundf(d->bamocar_current_actual * 10.0f));
	        CAN_data[0] = rpm & 0xFF;
	        CAN_data[1] = rpm>>8;
	        CAN_data[2] = torque & 0xFF;
	        CAN_data[3] = torque>>8;
	        CAN_data[4] = vdc & 0xFF;
	        CAN_data[5] = vdc>>8;
	        CAN_data[6] = current & 0xFF;
	        CAN_data[7] = current>>8;
	        HAL_CAN_AddTxMessage(AiM_bus_handle, &log_performance_tx, CAN_data, &message_box);}

void LoggerCAN_SendTemps(const CombinedLogData_t *tmps)
{
	        if (!AiM_bus_handle) return;

	        uint8_t  CAN_data[8] = {0};
	        uint32_t mbx;

	        // °C from your interpolation tables (already implemented in bamocar-can.c)
	        float   motorC = Bamocar_getMotorTemp(&myBamocar);
	        float   igbtC  = Bamocar_getControllerTemp(&myBamocar);

	        int16_t mot_x100  = clamp_i16((int32_t)lroundf(motorC * 100.0f));
	        int16_t igbt_x100 = clamp_i16((int32_t)lroundf(igbtC  * 100.0f));

	        CAN_data[0] = (uint8_t)(mot_x100 & 0xFF);
	        CAN_data[1] = (uint8_t)(mot_x100 >> 8);
	        CAN_data[2] = (uint8_t)(igbt_x100 & 0xFF);
	        CAN_data[3] = (uint8_t)(igbt_x100 >> 8);

	        HAL_CAN_AddTxMessage(AiM_bus_handle, &log_temperatures_tx, CAN_data, &mbx);
}

void LoggerCAN_SendFlags(const CombinedLogData_t *flgs)
{
	 if (!AiM_bus_handle || !flgs) {
	        fprintf(stderr,"%s","AiM bus handle or func param uninitialized");
	        return;
	    }

	    uint8_t CAN_data[8] = {0};
	    uint32_t mbx;

	    // Pack digital flags from CombinedLogData_t into CAN bytes
	    CAN_data[0] = (uint8_t)(flgs->main_brake_engaged   & 0x01);
	    CAN_data[1] = (uint8_t)(flgs->main_rtd_engaged     & 0x01);
	    CAN_data[2] = (uint8_t)(flgs->main_activation_engaged & 0x01);
	    CAN_data[3] = (uint8_t)(flgs->main_fan_state       & 0x01);
	    CAN_data[4] = (uint8_t)(flgs->main_pump_state      & 0x01);

	    // Remaining bytes unused/reserved
	    CAN_data[5] = 0;
	    CAN_data[6] = 0;
	    CAN_data[7] = 0;

	    // Transmit the frame to CAN2, StdId = 0x502 (AiM_CAN_ID_FLAGS)
	    HAL_CAN_AddTxMessage(AiM_bus_handle, &log_flags_tx, CAN_data, &mbx);
}
// aim.c
void LoggerCAN_Send_APPS(const CombinedLogData_t *d)
{
    if (!AiM_bus_handle || !d) return;

    uint8_t  CAN_data[8] = {0};
    uint32_t mbx;

    // Scale % to deci-percent (0..1000)
    int32_t p1_x10 = (int32_t)lroundf(d->main_pedal1_filtered * 10.0f);
    int32_t p2_x10 = (int32_t)lroundf(d->main_pedal2_filtered * 10.0f);
    if (p1_x10 < 0) {
        p1_x10 = 0;
    } else if (p1_x10 > 1000) {
        p1_x10 = 1000;
    }

    if (p2_x10 < 0) {
        p2_x10 = 0;
    } else if (p2_x10 > 1000) {
        p2_x10 = 1000;
    }

    // Byte layout (little-endian 16-bit):
    // [0..1] APPS1 *10 (uint16)
    // [2..3] APPS2 *10 (uint16)
    // [4]    flags: bit0=brake_engaged, bit1=R2D, bit2=activation
    // [5]    reserved (0)
    // [6..7] reserved (0)
    CAN_data[0] = (uint8_t)(p1_x10 & 0xFF);
    CAN_data[1] = (uint8_t)(p1_x10 >> 8);
    CAN_data[2] = (uint8_t)(p2_x10 & 0xFF);
    CAN_data[3] = (uint8_t)(p2_x10 >> 8);

    uint8_t flags = 0;
    flags |= (d->main_brake_engaged        & 0x01) << 0;
    flags |= (d->main_rtd_engaged          & 0x01) << 1;
    flags |= (d->main_activation_engaged   & 0x01) << 2;
    CAN_data[4] = flags;

    HAL_CAN_AddTxMessage(AiM_bus_handle, &log_apps_tx, CAN_data, &mbx);
}




void AiM_Initialize_CAN_Tx_Headers(CAN_HandleTypeDef *hcan) {
	AiM_bus_handle = hcan;

	CAN_TxHeaderTypeDef base;
	base.StdId = 0U;
	base.ExtId = 0U;
	base.IDE = CAN_ID_STD;      // 11-bit (CAN 2.0A)
	base.RTR = CAN_RTR_DATA;
	base.DLC = 8U;
	base.TransmitGlobalTime = DISABLE;

	log_performance_tx = base;
	log_performance_tx.StdId = AiM_CAN_ID_PERFORMANCE;
	log_temperatures_tx = base;
	log_temperatures_tx.StdId = AiM_CAN_ID_TEMPS;
	log_flags_tx = base;
	log_flags_tx.StdId = AiM_CAN_ID_FLAGS;

	log_bamo_currA_tx = base;
	log_bamo_currA_tx.StdId = AiM_CAN_ID_BAMO_CURR_A;
	log_bamo_currB_tx = base;
	log_bamo_currB_tx.StdId = AiM_CAN_ID_BAMO_CURR_B;
	log_bamo_foc_tx = base;
	log_bamo_foc_tx.StdId = AiM_CAN_ID_BAMO_FOC;
	log_bamo_volt_tx = base;
	log_bamo_volt_tx.StdId = AiM_CAN_ID_BAMO_VOLT;
	log_apps_tx = base;
	log_apps_tx.StdId = AiM_CAN_ID_APPS;
}
void AiM_CAN_Send_Performance_Data(const CombinedLogData_t *d) {
	if (!AiM_bus_handle || !d) {
		fprintf(stderr, "%s", "AiM bus handle or func param uninitialized");
		return;
	}
	uint8_t CAN_data[8] = { 0 };
	uint32_t message_box;
	int16_t rpm = clamp_i16((int32_t) lroundf(d->bamocar_rpm));
	int16_t torque = clamp_i16(
			(int32_t) lroundf(d->bamocar_torque_actual * 10000.0f)); //Multiply by 10k to convert to integer.
	uint16_t vdc = clamp_u16(
			(int32_t) lroundf((float) d->bamocar_dc_voltage_raw / 31.5f));
	int16_t current = clamp_i16(
			(int32_t) lroundf(d->bamocar_current_actual * 10.0f));
	CAN_data[0] = rpm & 0xFF;
	CAN_data[1] = rpm >> 8;
	CAN_data[2] = torque & 0xFF;
	CAN_data[3] = torque >> 8;
	CAN_data[4] = vdc & 0xFF;
	CAN_data[5] = vdc >> 8;
	CAN_data[6] = current & 0xFF;
	CAN_data[7] = current >> 8;
	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_performance_tx, CAN_data,
			&message_box);
}

void AiM_CAN_Send_Temperature_Data(const CombinedLogData_t *d) {
	(void) d; // not used anymore; temps come directly from Bamocar
	if (!AiM_bus_handle)
		return;

	uint8_t CAN_data[8] = { 0 };
	uint32_t mbx;

	// °C from your interpolation tables (already implemented in bamocar-can.c)
	float motorC = Bamocar_getMotorTemp(&myBamocar);
	float igbtC = Bamocar_getControllerTemp(&myBamocar);

	int16_t mot_x100 = clamp_i16((int32_t) lroundf(motorC * 100.0f));
	int16_t igbt_x100 = clamp_i16((int32_t) lroundf(igbtC * 100.0f));

	CAN_data[0] = (uint8_t) (mot_x100 & 0xFF);
	CAN_data[1] = (uint8_t) (mot_x100 >> 8);
	CAN_data[2] = (uint8_t) (igbt_x100 & 0xFF);
	CAN_data[3] = (uint8_t) (igbt_x100 >> 8);


	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_temperatures_tx, CAN_data, &mbx);
}

static inline void wr_i16(uint8_t *p, int16_t v) {
	p[0] = (uint8_t) (v & 0xFF);
	p[1] = (uint8_t) (v >> 8);
}  //Little Endian Function

void AiM_CAN_Send_BamocarCurrentsA(const BamocarData *bd) {
	if (!AiM_bus_handle || !bd)
		return;
	uint8_t d[8] = { 0 };
	uint32_t mbx;
	wr_i16(&d[0], bd->I_Q_CMD);
	wr_i16(&d[2], bd->I_Q_CMD_RAMP);
	wr_i16(&d[4], bd->I_ACTUAL);
	wr_i16(&d[6], bd->I_Q_ACTUAL);
	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_bamo_currA_tx, d, &mbx);
}

void AiM_CAN_Send_BamocarCurrentsB(const BamocarData *bd) {
	if (!AiM_bus_handle || !bd)
		return;
	uint8_t d[8] = { 0 };
	uint32_t mbx;
	wr_i16(&d[0], bd->I1_ACTUAL);
	wr_i16(&d[2], bd->I2_ACTUAL);
	wr_i16(&d[4], bd->I3_ACTUAL);
	wr_i16(&d[6], bd->I_ACTUAL_FILT);
	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_bamo_currB_tx, d, &mbx);
}

void AiM_CAN_Send_BamocarFOC(const BamocarData *bd) {
	if (!AiM_bus_handle || !bd)
		return;
	uint8_t d[8] = { 0 };
	uint32_t mbx;
	wr_i16(&d[0], bd->I_D_ACTUAL);
	wr_i16(&d[2], bd->I_Q_ERROR);
	wr_i16(&d[4], bd->I_D_ERROR);
	wr_i16(&d[6], bd->N_ACTUAL_FILT);
	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_bamo_foc_tx, d, &mbx);
}

void AiM_CAN_Send_BamocarVoltages(const BamocarData *bd) {
	if (!AiM_bus_handle || !bd)
		return;
	uint8_t d[8] = { 0 };
	uint32_t mbx;
	wr_i16(&d[0], bd->V_D);
	wr_i16(&d[2], bd->V_EMF);
	wr_i16(&d[4], bd->V_Q);
	// V_DC_ACTUAL is uint16 in your struct — treat as raw
	d[6] = (uint8_t) (bd->V_DC_ACTUAL & 0xFF);
	d[7] = (uint8_t) (bd->V_DC_ACTUAL >> 8);
	HAL_CAN_AddTxMessage(AiM_bus_handle, &log_bamo_volt_tx, d, &mbx);
}

void AiM_CAN_Send_Mapped_Torque_Value(const BamocarData *bd)
{
	if(!AiM_bus_handle || !bd) return;
	uint8_t d[8] = {0};
	uint32_t mbx;
	wr_i16(&d[0],bd->BAMOCAR_TORQUE_MAPPED_VALUE);
	HAL_CAN_AddTxMessage(AiM_bus_handle,&log_mapped_torque_value_tx,d,&mbx);
}
