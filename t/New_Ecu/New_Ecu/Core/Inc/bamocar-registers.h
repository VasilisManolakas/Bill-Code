/*
 * bamocar-registers.h
 *
 *  Created on: Sept 7, 2025
 *      Author: Vasilis Manolakas
 */

#ifndef Bamocar_registers_h
#define Bamocar_registers_h

// CAN-Relevant settings
#define STD_RX_ID        0x201   //The ID the MC will listen on
#define STD_TX_ID        0x181    //The ID the MC will answer with
#define N 39

// Control Registers
#define REG_ENABLE        0x51    //Disable or Enable transmission
#define REG_REQUEST       0x3D    //Transmission request

// Data Registers
#define REG_STATUS        0x40    //Status of device
#define REG_READY         0xE2    //State of the device
#define REG_ERROR_STATUS  0x8F    //Error status of device (RegID: 0x8F from Doc)

#define REG_N_ACTUAL      0x30    //RPM actual
#define REG_N_ACTUAL_FILT 0xA8
#define REG_N_CMD         0x31    //RPM command
#define REG_N_MAX         0xC8    //(SPEED_RPMMAX) Maximum rotation speed in turns per minute (Servo)

#define REG_I_Q_ACTUAL      0x27    //Active Current actual
#define REG_I_DEVICE      0xC6    //Current device
#define REG_I_200PC       0xD9    //Current 200 PC

#define REG_TORQUE        0x90    //Torque reference

#define REG_RAMP_ACC      0x35    //Ramp Acceleration command
#define REG_RAMP_DEC      0xED    //Ramp Deceleration command

// Voltage Registers
#define REG_V_OUTPUT_PHASE  0x8A //Output phase voltage CAN LOW (Vout RegID: 0x8a from Doc), RMS value to motor
#define REG_VOUT_MOTOR    REG_V_OUTPUT_PHASE //Alias for REG_V_OUTPUT_PHASE, for better readability.
#define REG_V_DC_ACTUAL   0xEB    //DC Voltage Actual (UDC RegID: 0xEB from Doc)
#define REG_V_BUS 	   	  0x66 //Bus Voltage (Vbus RegID: 0x66 from Doc)
#define REG_V_EMF 0x29 //CAN HIGH (Tx)
#define REG_V_Q REG_V_EMF //CAN LOW (Rx)
#define REG_V_D 0x2A //CAN LOW

#define BAMOCAR_DC_VOLTAGE_SCALE_400V (1.0f / 55.0f)

//Current Registers --New
#define REG_I_ACTUAL 0x20
#define REG_I_Q_CMD 0x26
#define REG_IQ_CMD_RAMP 0X22
#define REG_I1_ACTUAL 0x54
#define REG_I2_ACTUAL 0x55
#define REG_I3_ACTUAL 0X56
#define REG_I_ACTUAL_FILT 0x5F
#define REG_I_D_ACTUAL 0x28
#define REG_I_Q_ERROR 0x38
#define REG_I_D_ERROR 0x39



#define REG_TORQUE_OUTPUT_MOUT 0xA0 //Torque Output Mout (RegID: 0xA0 from Doc)

#define REG_TEMP_MOTOR    0x49    //Active motor temperature
#define REG_TEMP_IGBT     0x4A    //Active output stage temperature
#define REG_TEMP_AIR      0x4B    //Air temperature in the servo

#define REG_HARD_ENABLED  0xE8    //Hard Enabled State

// Request interval Pre-settings
#define INTVL_IMMEDIATE   0x00
#define INTVL_SUSPEND     0xFF
#define INTVL_100MS       0x64
#define INTVL_200MS       0xC8
#define INTVL_250MS       0xFA

// Torque Scaling
#define TORQUE_SCALING_FACTOR 0x5555

#define BAMOCAR_TX_ID_OPERATIONAL_DATA 0x186 // Operational data ID
#define BAMOCAR_TX_ID_TEMPERATURE_DATA 0x187 // Temperature data ID

#endif
 /* SRC_BAMOCAR_REGISTERS_H_ */
