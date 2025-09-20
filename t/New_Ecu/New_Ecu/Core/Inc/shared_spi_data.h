/*
 * shared_spi_data.h
 *
 *  Created on: Apr 14, 2025
 *      Author: savva
 */

#ifndef SHARED_SPI_DATA_H
#define SHARED_SPI_DATA_H

#include <stdint.h>
#include <stddef.h> // Required for offsetof

#pragma pack(push, 1)

typedef struct {
    // --- Metadata ---
    uint32_t sequence_num;      // Incremented by STM32 for each update
    uint16_t data_length;       // Should be sizeof(CombinedLogData_t)
    uint16_t checksum;          // CRC-16 checksum calculated by STM32

    // --- Main Controller Data (Added Here) ---
    float    main_pedal1_filtered;    // %
    float    main_pedal2_filtered;    // %
    uint16_t main_brake1_raw;         // Raw ADC from I2C Sensor 1
    uint16_t main_brake2_raw;         // Raw ADC from I2C Sensor 2
    uint8_t  main_brake_engaged;      // 1 if engaged, 0 otherwise
    uint8_t  main_rtd_engaged;        // 1 if Ready to Drive state active
    uint8_t  main_activation_engaged; // 1 if Activation Logic button pressed/latched
    uint8_t  main_fan_state;          // 1 if fan is ON
    uint8_t  main_pump_state;         // 1 if pump is ON
    int16_t  main_stepper_pos;        // Relative brake balance stepper position
    uint8_t  main_rotary_val;         // Value 0-10 (or as defined)

    // --- BMS Data ---
    float bms_pack_current;
    float bms_pack_inst_voltage;
    float bms_pack_soc;
    uint8_t bms_relay_state;
    float bms_pack_dcl;
    float bms_high_temp;
    float bms_low_temp;
    float bms_pack_summed_voltage;
    float bms_high_cell_voltage;
    float bms_low_cell_voltage;
    float bms_max_pack_dcl;
    float bms_max_pack_voltage;

    // --- Bamocar Data ---
    float bamocar_rpm;                // Calculated RPM? Or just send raw?
    int16_t bamocar_rpm_actual_raw;   // N_ACTUAL (0x30)
    int16_t bamocar_n_max_raw;        // N_MAX (0xC8)
    float bamocar_torque_actual;      // Calculated Torque %? Or send raw?
    int16_t bamocar_torque_raw;       // TORQUE (0x90 or 0xA0)
    float bamocar_current_actual;     // Calculated Current? Or send raw?
    uint16_t bamocar_i_actual_raw;    // I_ACTUAL (0x20)
    uint16_t bamocar_i_device_raw;    // I_DEVICE (0xC6)
    uint16_t bamocar_i_200pc_raw;     // I_200PC (0xD9)
    uint16_t bamocar_motor_temp_raw;  // TEMP_MOTOR (0x49) raw value
    uint16_t bamocar_controller_temp_raw; // TEMP_IGBT (0x4A) raw value
    uint16_t bamocar_air_temp_raw;    // TEMP_AIR (0x4B) raw value
    uint16_t bamocar_dc_voltage_raw;  // V_DC_ACTUAL (0xEB) raw value
    uint16_t bamocar_error_status_raw;// ERROR_STATUS (0x8F) raw value
    uint32_t bamocar_status_raw;      // STATUS (0x40) raw value
    uint8_t bamocar_hard_enabled;     // HARD_ENABLED (0xE8) 1=enabled

    // --- AIM Data ---
    float aim_g_sensor_x;
    float aim_g_sensor_y;
    float aim_g_sensor_z;
    float aim_pitch_angle;
    float aim_yaw_moment;
    uint8_t aim_gps_signal_valid; // Use uint8_t for bool
    float aim_gps_latitude;
    float aim_gps_longitude;
    float aim_gps_speed;
    float aim_steering_angle;
    float aim_brake_pressure;
    float aim_angleroll;
    float aim_distance;
    float aim_distance_lap;
    float aim_duration;
    float aim_pos_left_back_sensor;
    float aim_pos_right_back_sensor;
    float aim_pos_left_front_sensor;
    float aim_pos_right_front_sensor;
    float aim_pedal_position;
    float aim_brake_position;
    float aim_wheel_speed_rb;
    float aim_wheel_speed_lb;
    float aim_wheel_speed_rf;
    float aim_wheel_speed_lf;
    float aim_speed_velocity_rb;
    float aim_speed_velocity_lb;
    float aim_speed_velocity_rf;
    float aim_speed_velocity_lf;
    float aim_time; // System time from STM32? Or AIM time?

    // --- FUNC Data (Corrected prefix) ---
    float func_balance_brake_value;
    uint8_t func_engine_mode;

} CombinedLogData_t;

#pragma pack(pop) // --- Restore previous packing ---


#endif // SHARED_SPI_DATA_H
