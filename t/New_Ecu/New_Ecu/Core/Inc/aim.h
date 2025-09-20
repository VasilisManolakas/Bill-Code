/*
 * aim.h
 *
 *  Created on: Apr 13, 2025
 *      Author: savva
 */

#ifndef INC_AIM_H_
#define INC_AIM_H_

#include "stm32f4xx_hal.h" // Or appropriate HAL header
#include <stdint.h>
#include <stdbool.h>

#define AIM_SIMULATE_DATA // Uncomment to simulate data for testing
#define AIM_UPDATE_INTERVAL_MS 100 // Update interval in milliseconds

// --- AIM Data Structure ---
typedef struct {
    // Fields matching shared_spi_data.h (with aim_ prefix removed for internal use)
    float g_sensor_x;
    float g_sensor_y;
    float g_sensor_z;
    float pitch_angle;
    float yaw_moment;
    uint8_t gps_signal_valid; // Use uint8_t for bool compatibility
    float gps_latitude;
    float gps_longitude;
    float gps_speed;
    float steering_angle;
    float brake_pressure;
    float angleroll;
    float distance;
    float distance_lap;
    float duration;
    float pos_left_back_sensor;
    float pos_right_back_sensor;
    float pos_left_front_sensor;
    float pos_right_front_sensor;
    float pedal_position;
    float brake_position;
    float wheel_speed_rb;
    float wheel_speed_lb;
    float wheel_speed_rf;
    float wheel_speed_lf;
    float speed_velocity_rb; // Clarify if needed or derived from wheel speed
    float speed_velocity_lb;
    float speed_velocity_rf;
    float speed_velocity_lf;
    float time; // Source? System tick? GPS time?

    uint32_t last_update_time;
} AIM_Data_t;

// --- Function Prototypes ---
bool AIM_Init(void);
void AIM_UpdateData(void); // Poll sensors, check UART/CAN flags etc.
void AIM_ProcessMessage(uint8_t *rxData, uint8_t dlc, uint32_t msgId); // If using CAN/UART push
void AIM_ProcessGpsChar(uint8_t); // Example: called by UART ISR for each GPS char

// --- Getter Functions ---
// Add Getters for ALL fields needed by spi_comm.c
float AIM_GetGSensorX(void);
float AIM_GetGSensorY(void);
float AIM_GetGSensorZ(void);
float AIM_GetPitchAngle(void);
float AIM_GetYawMoment(void);
uint8_t AIM_GetGpsValid(void); // Returns 1 or 0
float AIM_GetGpsLatitude(void);
float AIM_GetGpsLongitude(void);
float AIM_GetGpsSpeed(void);
float AIM_GetSteeringAngle(void);
float AIM_GetBrakePressure(void);
float AIM_GetAngleroll(void);
float AIM_GetDistance(void);
float AIM_GetDistanceLap(void);
float AIM_GetDuration(void);
float AIM_GetPosLeftBack(void);
float AIM_GetPosRightBack(void);
float AIM_GetPosLeftFront(void);
float AIM_GetPosRightFront(void);
float AIM_GetPedalPosition(void);
float AIM_GetBrakePosition(void);
float AIM_GetWheelSpeedRB(void);
float AIM_GetWheelSpeedLB(void);
float AIM_GetWheelSpeedRF(void);
float AIM_GetWheelSpeedLF(void);
float AIM_GetSpeedVelocityRB(void);
float AIM_GetSpeedVelocityLB(void);
float AIM_GetSpeedVelocityRF(void);
float AIM_GetSpeedVelocityLF(void);
float AIM_GetTime(void);

#endif /* INC_AIM_H_ */
