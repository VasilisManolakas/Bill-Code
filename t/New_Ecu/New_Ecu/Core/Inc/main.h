/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APPS1_STM_Pin GPIO_PIN_1
#define APPS1_STM_GPIO_Port GPIOA
#define APPS2_STM_Pin GPIO_PIN_2
#define APPS2_STM_GPIO_Port GPIOA
#define DRS_Pin GPIO_PIN_3
#define DRS_GPIO_Port GPIOA
#define FAN_CONTROL_Pin GPIO_PIN_4
#define FAN_CONTROL_GPIO_Port GPIOC
#define FAN_BUTTON_Pin GPIO_PIN_5
#define FAN_BUTTON_GPIO_Port GPIOC
#define PUMP_CONTROL_Pin GPIO_PIN_0
#define PUMP_CONTROL_GPIO_Port GPIOB
#define PUMP_BUTTON_Pin GPIO_PIN_1
#define PUMP_BUTTON_GPIO_Port GPIOB
#define BRAKE_L_C_Pin GPIO_PIN_2
#define BRAKE_L_C_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOB
#define A_L_BUTTON_Pin GPIO_PIN_14
#define A_L_BUTTON_GPIO_Port GPIOB
#define R2D_BUTTON_Pin GPIO_PIN_15
#define R2D_BUTTON_GPIO_Port GPIOB
#define FRG_CONTROL_Pin GPIO_PIN_6
#define FRG_CONTROL_GPIO_Port GPIOC
#define SPEAKER_CONTROL_Pin GPIO_PIN_7
#define SPEAKER_CONTROL_GPIO_Port GPIOC
#define AL_2_Pin GPIO_PIN_8
#define AL_2_GPIO_Port GPIOC
#define AL_1_Pin GPIO_PIN_9
#define AL_1_GPIO_Port GPIOC
#define DIR_BB_Pin GPIO_PIN_9
#define DIR_BB_GPIO_Port GPIOA
#define STEP_BB_Pin GPIO_PIN_10
#define STEP_BB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define CONTROL_BJT_1_Pin GPIO_PIN_10
#define CONTROL_BJT_1_GPIO_Port GPIOC
#define CONTROL_BJT_2_Pin GPIO_PIN_11
#define CONTROL_BJT_2_GPIO_Port GPIOC
#define OE_LVL_SHFT_Pin GPIO_PIN_2
#define OE_LVL_SHFT_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define BRAKE_PEDAL_Pin GPIO_PIN_4
#define BRAKE_PEDAL_GPIO_Port GPIOB
#define DRS_LED_Pin GPIO_PIN_6
#define DRS_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// Debounce & Timers
#define DEBOUNCE_DELAY_MS           (50)
#define R2D_SPEAKER_DURATION_MS     (3000)

#define PRECHARGE_VOLTAGE_TARGET_PERCENT (0.91f)

#define MAX_STEPS_LEFT 10          // Maximum steps to the left
#define MAX_STEPS_RIGHT 10         // Maximum steps to the right
#define STEPS_PER_ROTARY_INCREMENT 100 // Steps per rotary switch increment

// Brake sensor defines - ** MUST CONFIGURE **
#define BRAKE_SENSOR1_ADDRESS 0x4D // Replace 0x4D with the actual I2C address of your first brake sensor (e.g., 0x4D, 0x4E, etc.)
#define BRAKE_SENSOR2_ADDRESS 0x4E // Replace 0x4E with the actual I2C address of your second brake sensor (e.g., 0x4D, 0x4E, etc.)
#define BRAKE_VALUE_REG 0x00       // Register to read brake value from MCP3221 (Likely 0x00 - check datasheet)
#define BRAKE_THRESHOLD 200        // Example threshold value - adjust based on sensor readings
#define BRAKE_VALUES_SIZE 2        // Size of each brake value in bytes (MCP3221 is 12-bit, so 2 bytes)

#define MOVING_AVG_SIZE 10
#define BRAKE_PEDAL_THRESHOLD 0.70f // Voltage threshold for brake pedal engagement (0.6V)
//PART NO.		BIN			HEX		DEC	MARKING
//MCP3221A0T-E/OT	01001000	0x48	72	GE
//MCP3221A1T-E/OT	01001001	0x49	73	GH
//MCP3221A2T-E/OT	01001010	0x4A	74	GB
//MCP3221A3T-E/OT	01001000	0x4B	75	GC
//MCP3221A4T-E/OT	01001100	0x4C	76	GD
//MCP3221A5T-E/OT	01001101	0x4D	77	GA
//MCP3221A6T-E/OT	01001110	0x4E	78	GF
//MCP3221A7T-E/OT	01001111	0x4F	79	GG

// ADC Configuration (Assuming 3.3V Reference, 12-bit ADC)
#define ADC_VREF                (3.3f)
#define ADC_MAX_VALUE           (4095) // 2^12 - 1

// Sensor 1 ( ~1270 to 2915 ADC)
#define APPS1_MIN_VOLTAGE       (0.950f) // Minimum voltage for APPS1
#define APPS1_MAX_VOLTAGE       (2.255f) // Maximum voltage for APPS1
#define APPS1_MIN_ADC_RAW            (uint16_t)((APPS1_MIN_VOLTAGE / ADC_VREF) * ADC_MAX_VALUE) // ~2083
#define APPS1_MAX_ADC_RAW            (uint16_t)((APPS1_MAX_VOLTAGE / ADC_VREF) * ADC_MAX_VALUE) // ~4095

// Sensor 2 (0.843V to 1.635V -> ~1046 to 2029 ADC)
#define APPS2_MIN_VOLTAGE       (0.480f)
#define APPS2_MAX_VOLTAGE       (1.112f)
#define APPS2_MIN_ADC_RAW            (uint16_t)((APPS2_MIN_VOLTAGE / ADC_VREF) * ADC_MAX_VALUE) // ~1046
#define APPS2_MAX_ADC_RAW            (uint16_t)((APPS2_MAX_VOLTAGE / ADC_VREF) * ADC_MAX_VALUE) // ~2029

// APPS Plausibility Check Configuration
#define APPS_PLAUSIBILITY_THRESHOLD_PERCENT (10.0f) // 10% difference threshold
#define APPS_IMPLAUSIBILITY_TIMEOUT_MS      (100)   // 100 ms persistence time
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
