/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>	//C Standard Input - Output lib
#include <stdlib.h>	//C Standard Library Inclusion
#include <math.h>	//Cmath
#include <stdbool.h>	//Boolean Handler library
#include "FIRFilter.h"	//Custom Finite Impulse Response Filter Library inclusion (FIR Initialization & Value Computing)

// Files for CAN communication
#include "bamocar-can.h"
#include "bms.h"
#include "aim.h"
#include "shared_spi_data.h"
#include "func.h"
#include "aim-registers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// VEHICLE STATE MACHINE DEFINITION, i.e. Vehicle Modes. This Enumeration/Struct is used for vehicle state detection.
typedef enum {
	VEHICLE_STATE_OFF,
	VEHICLE_STATE_ACTIVATION_BUTTON_PRESSED,
	VEHICLE_STATE_PRECHARGING,
	VEHICLE_STATE_ACTIVATED,
	VEHICLE_STATE_R2D_PROCEDURE_START,
	VEHICLE_STATE_R2D_PROCEDURE_SPEAKER,
	VEHICLE_STATE_READY_TO_DRIVE,
	VEHICLE_STATE_APPS_FAULT,	//Accelerator Pedal Position Sensor Error Flag
	VEHICLE_STATE_BAMOCAR_FAULT,
	VEHICLE_STATE_BMS_FAULT
// Add other fault states as needed
} VehicleState_t;

typedef struct {//Struct intended for a digital signal filtering (moving average filters).
	uint16_t buffer[MOVING_AVG_SIZE];
	uint32_t sum;  // Use 32-bit to prevent overflow when summing 16-bit values
	uint8_t index; // Current position in the circular buffer
	uint8_t count; // Number of samples currently in the buffer (for initial fill)
} MovingAverageFilter_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define for the shutdown circuit voltage check
#define SHUTDOWN_CIRCUIT_VOLTAGE_THRESHOLD 60.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// --- Vehicle State ---
volatile VehicleState_t currentVehicleState = VEHICLE_STATE_OFF;
uint32_t stateTimer = 0; // Generic timer for states if needed

// --- Shutdown Circuit Logic ---
// This flag tracks if the Tractive System has been energized (>60V)
volatile bool high_voltage_was_active_flag = false;

// --- Button States & Timers (Used by new state machine) ---
uint32_t precharge_start_time = 0;
uint32_t lastALButtonDebounceTime = 0;
uint8_t alButtonStateDebounced = 0; // 0 = released, 1 = pressed

uint32_t lastR2DButtonDebounceTime = 0;
uint8_t r2dButtonStateDebounced = 0;

// Rotary switch control
volatile int rotarySwitchValue = 0; // Rotary switch value from CAN (0-10)
int currentRotarySwitchValue = -1; // To detect changes and move stepper only on change
volatile int currentStepperPosition = 0; // Tracks the current stepper position relative to center (0 = center)

// Fan control
volatile GPIO_PinState fanOutputState = GPIO_PIN_RESET;	//Make sure the fan is off.
uint32_t lastFanButtonPressTime = 0; // To store the last button press time for debouncing
uint8_t fanButtonPressedRaw = 0; // Flag to indicate if the button is currently pressed

// Pump control
volatile GPIO_PinState pumpOutputState = GPIO_PIN_SET;//Make sure the pumps are on.
uint32_t lastPumpButtonPressTime = 0; // To store the last button press time for debouncing
uint8_t pumpButtonPressedRaw = 0;

//Brake values
volatile bool brakeEngagedPhysically = false; // Renamed for clarity
uint8_t i2c_brakeValue1Raw[BRAKE_VALUES_SIZE]; // Renamed to avoid conflict
uint8_t i2c_brakeValue2Raw[BRAKE_VALUES_SIZE];
volatile uint16_t processedBrakeValue1; // Renamed
volatile uint16_t processedBrakeValue2;
float brakeSensor1Voltage;

//Activation Logic Variables
uint32_t lastALButtonPressTime = 0; // To store the last button press time for debouncing
uint8_t ALButtonPressed = 0;
volatile bool activationEngaged = false; // Track if Activation Logic is engaged

//Ready to Drive Logic Variables
uint32_t lastR2DButtonPressTime = 0; // To store the last button press time for debouncing
uint8_t R2DButtonPressed = 0;
volatile bool readyToDriveEngaged = false; // Track Ready to Drive state
GPIO_PinState AL1State = GPIO_PIN_RESET; // AL1 State
GPIO_PinState AL2State = GPIO_PIN_RESET; // AL2 State
GPIO_PinState frgState = GPIO_PIN_RESET; // Function Ready Group State
GPIO_PinState speakerState = GPIO_PIN_RESET; // Speaker State
uint32_t speakerStartTime = 0; // Start time for speaker timer
bool speakerActive = false;     // Speaker active flag
bool speakerPlayed = false; // Flag to indicate if speaker has played for current R2D press
float voltageInverter = 0.0f;   // Voltage Inverter value from CAN
float voltageBattery = 1.0f;    // Voltage Battery value from CAN
bool voltageCheckPassed = false; // Voltage check flag

// Bamocar instance

Bamocar myBamocar;
CAN_TxHeaderTypeDef bamocarTxHeader;
CAN_RxHeaderTypeDef bamocarRxHeader;
uint8_t bamocarTxData[8]; // Increased size just in case, although Bamocar uses less
uint8_t bamocarRxData[8];
uint32_t bamocarTxMailbox;

BMS_Data_t bms_data;
CAN_RxHeaderTypeDef bmsRxHeader; // BMS Rx Header
uint8_t bmsRxData[8]; // BMS Rx Data buffer
uint32_t bmsRxMailbox; // BMS Rx Mailbox

AIM_Data_t aim_data;   // Assumed updated by CAN callbacks
FUNC_Data_t func_data; // Assumed updated by CAN callbacks

// --- SPI Communication ---
CombinedLogData_t dataToSend; // Using the new struct from shared_spi_data.h
volatile bool spi_transfer_complete_flag = true; // Renamed
static uint32_t spi_tx_sequence_counter = 0;    // Renamed
uint8_t spiTxBuffer[sizeof(CombinedLogData_t)]; // Transmit buffer (Consider making static if large)

// --- ADC & APPS ---
volatile uint16_t adc1ResultDma[2]; // pedal1_raw, pedal2_raw
const int adc1ChannelCount = sizeof(adc1ResultDma) / sizeof(adc1ResultDma[0]);
volatile bool adcConversionCompleteFlag = false; // Renamed from AdcComplete
FIRFilter filterPedalSensor1;
FIRFilter filterPedalSensor2;
FIRFilter filterBrakeSensor;
volatile float apps1_percentage_filtered = 0.0f; // Renamed
volatile float apps2_percentage_filtered = 0.0f;
// APPS Plausibility (used by new state machine)
volatile bool apps_implausibility_pending_flag = false; // Renamed
volatile uint32_t apps_implausibility_timer_start_ms = 0;

// --- Outputs Controlled by State Machine ---
GPIO_PinState frgOutputPinActualState = GPIO_PIN_RESET;
GPIO_PinState al1OutputPinActualState = GPIO_PIN_RESET;
GPIO_PinState al2OutputPinActualState = GPIO_PIN_RESET;
GPIO_PinState speakerOutputPinActualState = GPIO_PIN_RESET;
uint32_t speakerOnTimeStart_ms = 0;
bool speakerHasPlayedForCurrentR2D = false;

// These are specific values needed by the state machine logic
volatile float bms_current_pack_voltage = 0; // Default, MUST be updated by BMS_ProcessCAN
volatile bool bms_is_fault_active = false; // Default, MUST be updated by BMS_ProcessCAN

int counter = 0; // General counter for various purposes
uint32_t current_tick_ms = 0; // Current tick in milliseconds

bool al_button_physical_state = false;    // Assuming Active HIGH
bool r2d_button_physical_state = false;   // Assuming Active LOW
bool fan_button_physical_state = false;
bool pump_button_physical_state = false;

uint16_t desired_torque_to_bamocar = 0; // Value from -1.0 to 1.0 for Bamoca
float current_bamocar_dc_bus_voltage = 0;

//Motor temperature variables
volatile uint16_t motorTemp;
float controllerTemp;

// Brake sensor raw values
volatile uint16_t brakeSensor1Raw = 0; // Brake sensor 1 raw value
uint16_t filtered_brakeSensor1Voltage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t calculateCRC16(const uint8_t *data, size_t length);
void Process_APPS_Readings_And_Filter(uint16_t raw_adc1, uint16_t raw_adc2);
bool Check_APPS_Implausibility(float percent1_filt, float percent2_filt);
HAL_StatusTypeDef Read_I2C_Brake_Sensors(void);
void Process_I2C_Brake_Data(void);
void Check_For_System_Overrides(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Delays for a specified number of microseconds using TIM1.
 * @param  delay: Delay in microseconds.
 * @retval None
 */
void microDelay(uint16_t delay) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < delay)
		;
}

/**
 * @brief  Steps the stepper motor a specified number of steps.
 * @param  steps: Number of steps to take.
 * @param  direction: 0 for left (CCW), 1 for right (CW).
 * @param  delay: Delay in microseconds between steps (adjust speed).
 * @retval None
 */
void step(int steps, uint8_t direction, uint16_t delay) {
	int x;
	if (direction == 0) // Left (CCW)
		HAL_GPIO_WritePin(DIR_BB_GPIO_Port, DIR_BB_Pin, GPIO_PIN_RESET); // Assuming DIR pin LOW for left
	else
		// Right (CW)
		HAL_GPIO_WritePin(DIR_BB_GPIO_Port, DIR_BB_Pin, GPIO_PIN_SET); // Assuming DIR pin HIGH for right

	for (x = 0; x < steps; x = x + 1) {
		HAL_GPIO_WritePin(STEP_BB_GPIO_Port, STEP_BB_Pin, GPIO_PIN_SET); // Step pulse HIGH
		microDelay(delay);
		HAL_GPIO_WritePin(STEP_BB_GPIO_Port, STEP_BB_Pin, GPIO_PIN_RESET); // Step pulse LOW
		microDelay(delay);
	}
}

/**
 * @brief Moves the stepper motor to the position corresponding to the rotary switch value.
 * @param rotaryValue: Value from the rotary switch (0-10).
 * @retval None
 */
void moveStepperToPosition(int rotaryValue) {	//Steppper motor move function
	int targetPosition = 0; // Target position in steps (relative to center)
	int stepsToMove = 0;
	uint8_t direction = 0; // Default direction

	if (rotaryValue == 0) {
		targetPosition = 0; // Middle position is target
	} else if (rotaryValue >= 1 && rotaryValue <= 5) {
		targetPosition = -rotaryValue * STEPS_PER_ROTARY_INCREMENT; // Left positions are negative
		if (targetPosition < -MAX_STEPS_LEFT)
			targetPosition = -MAX_STEPS_LEFT; // Limit left travel
	} else if (rotaryValue >= 6 && rotaryValue <= 10) {
		targetPosition = (rotaryValue - 5) * STEPS_PER_ROTARY_INCREMENT; // Right positions are positive
		if (targetPosition > MAX_STEPS_RIGHT)
			targetPosition = MAX_STEPS_RIGHT;  // Limit right travel
	} else {
		return; // Invalid rotary value, do nothing
	}

	stepsToMove = abs(targetPosition - currentStepperPosition); // Calculate steps needed from current position

	if (targetPosition > currentStepperPosition) {
		direction = 1; // Move right (CW)
	} else if (targetPosition < currentStepperPosition) {
		direction = 0; // Move left (CCW)
	} else {
		stepsToMove = 0; // Already at target position, no movement needed
	}

	if (stepsToMove > 0) {
		step(stepsToMove, direction, 1000); // Move calculated steps, adjust delay for speed
	}
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax,
		uint32_t au32_OUTmin, uint32_t au32_OUTmax) {
	return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
			/ (au32_INmax - au32_INmin)) + au32_OUTmin);
}

uint16_t update_moving_average(MovingAverageFilter_t *filter,//Moving average signal filtering
		uint16_t new_value) {
	if (filter->count == MOVING_AVG_SIZE) {
		filter->sum -= filter->buffer[filter->index];
	}
	filter->buffer[filter->index] = new_value;
	filter->sum += new_value;
	filter->index = (filter->index + 1) % MOVING_AVG_SIZE;
	if (filter->count < MOVING_AVG_SIZE) {
		filter->count++;
	}
	if (filter->count == 0)
		return 0;
	return (uint16_t) (filter->sum / filter->count);
}

// --- APPS Data Processing & Plausibility ---
void Process_APPS_Readings_And_Filter(uint16_t raw_adc1, uint16_t raw_adc2) {
	float adc1_clamped = (float) raw_adc1;
	float adc2_clamped = (float) raw_adc2;
	float p1_percent_unfiltered = 0.0f;
	float p2_percent_unfiltered = 0.0f;

	// Clamp & Calculate Percentage Sensor 1
	if (adc1_clamped < APPS1_MIN_ADC_RAW)
		adc1_clamped = APPS1_MIN_ADC_RAW;
	else if (adc1_clamped > APPS1_MAX_ADC_RAW)
		adc1_clamped = APPS1_MAX_ADC_RAW;
	p1_percent_unfiltered = ((adc1_clamped - APPS1_MIN_ADC_RAW ) * 100.0f)
			/ (float) (APPS1_MAX_ADC_RAW - APPS1_MIN_ADC_RAW );

	// Clamp & Calculate Percentage Sensor 2
	if (adc2_clamped < APPS2_MIN_ADC_RAW)
		adc2_clamped = APPS2_MIN_ADC_RAW;
	else if (adc2_clamped > APPS2_MAX_ADC_RAW)
		adc2_clamped = APPS2_MAX_ADC_RAW;
	p2_percent_unfiltered = ((adc2_clamped - APPS2_MIN_ADC_RAW ) * 100.0f)
			/ (float) (APPS2_MAX_ADC_RAW - APPS2_MIN_ADC_RAW );

	FIRFilter_Update(&filterPedalSensor1, p1_percent_unfiltered);
	FIRFilter_Update(&filterPedalSensor2, p2_percent_unfiltered);

	apps1_percentage_filtered = filterPedalSensor1.out;
	apps2_percentage_filtered = filterPedalSensor2.out;
}

bool Check_APPS_Implausibility(float percent1_filt, float percent2_filt) {
	// This function will be called only if not already in APPS_FAULT state
	float deviation = fabsf(percent1_filt - percent2_filt);

	if (deviation > APPS_PLAUSIBILITY_THRESHOLD_PERCENT) {
		if (!apps_implausibility_pending_flag) {
			apps_implausibility_pending_flag = true;
			apps_implausibility_timer_start_ms = HAL_GetTick();
		} else {
			if ((HAL_GetTick() - apps_implausibility_timer_start_ms)
					> APPS_IMPLAUSIBILITY_TIMEOUT_MS) {
				apps_implausibility_pending_flag = false; // Reset pending once fault is to be latched
				return true; // Implausibility detected and persisted
			}
		}
	} else {
		apps_implausibility_pending_flag = false; // Deviation within tolerance
	}
	return false; // No implausibility to latch this cycle
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		adcConversionCompleteFlag = true; // Set flag for main loop to process
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {	//Debug
	if (hspi == &hspi1) {
		// Basic error handling: reset transfer flag to retry
		spi_transfer_complete_flag = true;
		// Consider logging the error or resetting SPI peripheral
	}
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		spi_transfer_complete_flag = true;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Attempt to retrieve the message from the FIFO
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &bamocarRxHeader, bamocarRxData);

	// Process messages from the Powertrain bus (CAN1)
	if (hcan->Instance == CAN1) {
		// Use a switch statement to route messages based on their CAN ID
		switch (bamocarRxHeader.StdId) {
		case 0x181: // Message from Bamocar Inverter
			Bamocar_parseMessage(&myBamocar, &bamocarRxHeader, bamocarRxData);
			break;

			// These cases all fall through to the same handler
		case BMS_ID_VOLT:   // Message with BMS Voltage Data (0x182)
		case BMS_ID_MAIN:   // Message with Main BMS Data (0x183)
		case BMS_ID_LIMITS: // Message with BMS Limits Data (0x184)
		case BMS_ID_TEMP:   // Message with BMS Temperature Data (0x185)
			BMS_ProcessMessage(bamocarRxData, bamocarRxHeader.DLC,
					bamocarRxHeader.StdId);
			break;

		default:
			// Optional: Code to handle any other unexpected CAN ID on this bus
			break;
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_CRC_Init();
	MX_I2C1_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */

	//Enable pump
	HAL_GPIO_WritePin(PUMP_CONTROL_GPIO_Port, PUMP_CONTROL_Pin, GPIO_PIN_SET); // Enable Pump Control

	//TIM1
	HAL_TIM_Base_Start(&htim1); // Start TIM1 for microsecond delay

	FIRFilter_Init(&filterPedalSensor1);
	FIRFilter_Init(&filterPedalSensor2);
	FIRFilter_Init(&filterBrakeSensor);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);



	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	AiM_Initialize_CAN_Tx_Headers(&hcan2);	//Initialize Data Logger CAN
	// Bamocar Initialization	- Inverter
	Bamocar_init(&myBamocar, STD_RX_ID, STD_TX_ID, &bamocarTxHeader,
			&bamocarRxHeader, bamocarTxData, bamocarRxData, &bamocarTxMailbox,
			&hcan1);
	Bamocar_subscribeFSFields(&myBamocar, INTVL_100MS);


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1ResultDma, adc1ChannelCount);

	BMS_Init();
	AIM_Init();
	FUNC_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		current_tick_ms = HAL_GetTick();

		// Heartbeat LED every 500ms

		if (current_tick_ms > (stateTimer + 500)) {
			stateTimer = current_tick_ms;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}

		// Read adc2 - Brake Sensors

		if (HAL_ADC_Start(&hadc2) != HAL_OK) {
			Error_Handler(); // Handle ADC start error
		}

		HAL_ADC_PollForConversion(&hadc2, 100); // Poll for conversion completion
		brakeSensor1Raw = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);

		// Convert raw ADC value to an unfiltered voltage (assuming 12-bit ADC and 3.3V ref)
		brakeSensor1Voltage = (brakeSensor1Raw / 4095.0f) * 3.3f;

		// Apply the FIR filter to get the clean, final voltage
		filtered_brakeSensor1Voltage = FIRFilter_Update(&filterBrakeSensor,
				brakeSensor1Voltage);

		// Clamp the filtered value to 0 to prevent any negative values due to filter ringing
		if (brakeSensor1Voltage < 0.0f) {
			brakeSensor1Voltage = 0.0f;
		}

		// APPS (ADC DMA) - Process if new data is available
		if (adcConversionCompleteFlag) {
			adcConversionCompleteFlag = false; // Reset flag
			Process_APPS_Readings_And_Filter(adc1ResultDma[0],
					adc1ResultDma[1]);
			if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1ResultDma,
					adc1ChannelCount) != HAL_OK) {
				Error_Handler(); // Handle ADC restart error
			}
		}

		// --- 1. Read Physical Inputs ---
		al_button_physical_state = !HAL_GPIO_ReadPin(A_L_BUTTON_GPIO_Port,
		A_L_BUTTON_Pin);   // Assuming Active HIGH
		if (!HAL_GPIO_ReadPin(R2D_BUTTON_GPIO_Port, R2D_BUTTON_Pin)) {
			r2d_button_physical_state = true;   // Assuming Active LOW
		}
		fan_button_physical_state = HAL_GPIO_ReadPin(FAN_BUTTON_GPIO_Port,
		FAN_BUTTON_Pin);
		pump_button_physical_state = HAL_GPIO_ReadPin(PUMP_BUTTON_GPIO_Port,
		PUMP_BUTTON_Pin);

		//brakeEngagedPhysically = HAL_GPIO_ReadPin(BRAKE_PEDAL_GPIO_Port, BRAKE_PEDAL_Pin); // Assuming Active LOW for brake pedal
		if (filterBrakeSensor.out > BRAKE_PEDAL_THRESHOLD) {
			brakeEngagedPhysically = true; // Brake engaged if raw value is below threshold
		} else {
			brakeEngagedPhysically = false; // Brake not engaged
		}

		// Update brake light immediately based on physical input
		HAL_GPIO_WritePin(BRAKE_L_C_GPIO_Port, BRAKE_L_C_Pin,
				brakeEngagedPhysically ? GPIO_PIN_SET : GPIO_PIN_RESET);

		// --- 2. Update Data from External ECUs ---
		current_bamocar_dc_bus_voltage = Bamocar_getBusVoltage(&myBamocar);
		bms_current_pack_voltage = BMS_GetPackSummedVoltage();

		// --- 3. Check for Faults and System Overrides ---
		// This function will check for APPS/BMS/Inverter faults and also handle the
		// shutdown circuit logic (voltage dropping below 60V).
		Check_For_System_Overrides();

		// check if the temperature is bigger than 70 C enable FAN PUMP

		motorTemp = Bamocar_getMotorTemp(&myBamocar);
		controllerTemp = Bamocar_getControllerTemp(&myBamocar);

		// --- 4. Run State Machine & Determine Outputs ---
		switch (currentVehicleState) {
		case VEHICLE_STATE_OFF: //Case 0 : VEHICLE_STATE_OFF Means the HV TS Is not energized, therefore the Vehicle is not operating.
			frgOutputPinActualState = GPIO_PIN_RESET; //FRG = Front Relay Group, meaning a batch of Relays that are normally closed when the TS is not active. Does pack voltage go to the motor (and any other HV loads)?
			//when OFF, everything stays safe and isolated; when ON, you’re truly ready to drive.
			al1OutputPinActualState = GPIO_PIN_RESET; // First Pin (Precharge Pin) Wired to the coil of a high voltage contact set to logical 0.
			al2OutputPinActualState = GPIO_PIN_RESET; // Second Pin (main contact) Wired to the coil of a high voltage contact set to logical 0. When the Coil gets powered, it closes and lets voltage and current flow through.
			speakerOutputPinActualState = GPIO_PIN_RESET; //Speaker is Turned Off.
			speakerHasPlayedForCurrentR2D = false;//R2D Speaker Noise HAS Played Flag set to false, cause the vehicle's TS is off therefore the vehicle is not in Ready-To-Drive Conditio and the noise hasn't played .
			Bamocar_setSoftEnable(&myBamocar, false);//Kill the Inverter (Sends CAN command to clear the soft enable bit) .
			Bamocar_setTorque(&myBamocar, 0.0f);// Changes the inverter's torque to 0 Nm since the Vehicle is not operating.

			// Reset the shutdown logic flag for the next activation cycle.
			high_voltage_was_active_flag = false; //>60V DC or > 50V RMS Was detected = false.

			if (al_button_physical_state) { // Did the driver Press the Activation logic button?
				if (!alButtonStateDebounced	//Meaning the Button Press hasn't been handled
						&& (current_tick_ms - lastALButtonDebounceTime
								> DEBOUNCE_DELAY_MS)) { //The time for another debounce check has passed
					alButtonStateDebounced = 1; //Dont handle any more button presses until released.
					lastALButtonDebounceTime = current_tick_ms; //Set the Exact time Recorded in ms as the last time the Activation Logic Button was Debounced.
					currentVehicleState =
							VEHICLE_STATE_ACTIVATION_BUTTON_PRESSED; //Vehicle set to AL button pressed (meaning enum case 1)
				}
			} else { //If the Driver Didnt Press the AL Button the Button is released. (Not Debounced = Released , Debounced = Pressed)
				alButtonStateDebounced = 0;  //Button is released.
			}
			break;

		case VEHICLE_STATE_ACTIVATION_BUTTON_PRESSED: //Case 1 : VEHICLE_STATE_ACTIVATION_BUTTON_PRESSED Means the AL button is pressed.
			al1OutputPinActualState = GPIO_PIN_SET;	//First Pin (Precharge Pin) Wired to the coil of a high voltage contact set to logical 1.
			al2OutputPinActualState = GPIO_PIN_RESET;//Second Pin (main contact) Wired to the coil of a high voltage contact set to logical 0.
			frgOutputPinActualState = GPIO_PIN_RESET;//The Relays are the last step before the inverter. We're Still in the Pre-Charge state meaning only the AL1 pin must close the precharge contact allowing current.
			speakerOutputPinActualState = GPIO_PIN_RESET; //Speakers are still dead
			Bamocar_setSoftEnable(&myBamocar, false);	//inverter is still dead

			if (!al_button_physical_state) { // Button released cause we're still in pre-charge state. 
				alButtonStateDebounced = 0; //Boolean button state flag set to button = released as the if statement states.
				currentVehicleState = VEHICLE_STATE_PRECHARGING;
				precharge_start_time = current_tick_ms; // Start precharge timer (The time in miliseconds the pre charge current started to flow through is now.)
			}
			break;

		case VEHICLE_STATE_PRECHARGING:	//Case 2: VEHICLE_STATE_PRECHARGING Means the Vehicle has entered pre-charge state.

			if (((current_tick_ms - precharge_start_time) > 4000) //If the time elapsed since the vehicle entered pre-charge state and the inverter's dc bus voltage is less than the SDC's voltage threshold (50 V DC / 60V RMS) .
			//Based on the Rules it should be <5s, hence the check.
					&& (current_bamocar_dc_bus_voltage
							< SHUTDOWN_CIRCUIT_VOLTAGE_THRESHOLD)) {
				currentVehicleState = VEHICLE_STATE_BAMOCAR_FAULT; //Then an inverter fault is set as the vehicle's current state meaning the circumstances are not met and aren't optimal.
				break;
			}

			if (current_bamocar_dc_bus_voltage //DC Voltage as Read from the inverter via CAN
			>= (bms_current_pack_voltage * PRECHARGE_VOLTAGE_TARGET_PERCENT)
					&& bms_current_pack_voltage != 0) { //ensures that the voltage at the vehicle side of the AIRs is pre-charged to at least 95 % of the actual TS accumulator voltage before closing the second AIR
					// Precharge delay 7 seconds
//			if ((current_tick_ms - precharge_start_time) >= 7000) {
				currentVehicleState = VEHICLE_STATE_ACTIVATED; //Vehicle is set as activated. (the vehicle is safely pre - charged.)
				al2OutputPinActualState = GPIO_PIN_SET; // Activation successful, close main contactor
				speakerHasPlayedForCurrentR2D = false; // Reset for R2D sequence
			}
			break;

		case VEHICLE_STATE_ACTIVATED://Case 3: The vehicle is readying up to be activated.
			al1OutputPinActualState = GPIO_PIN_SET; //Close pre - charge contactor
			al2OutputPinActualState = GPIO_PIN_SET;	 //Close main contactor
			frgOutputPinActualState = GPIO_PIN_RESET;	//Open front relay group
			speakerOutputPinActualState = GPIO_PIN_RESET; //Speakers still dead.
			Bamocar_setSoftEnable(&myBamocar, false);	//Inverter Still dead.

			if (r2d_button_physical_state && brakeEngagedPhysically) {//If the ready - to - drive button is pressed and the brake is actuated (according to the rules)
				if (!r2dButtonStateDebounced	//If the button is not released 
				&& (current_tick_ms - lastR2DButtonDebounceTime	//AND the time for another debounce check has passed :
				> DEBOUNCE_DELAY_MS)) {
					r2dButtonStateDebounced = 1;//Button is being pressed at the moment.
					lastR2DButtonDebounceTime = current_tick_ms;// Last time it was pressed was now.
					currentVehicleState = VEHICLE_STATE_R2D_PROCEDURE_START;//Vehicle enters the ready to drive procedure.
				}
			} else {
				r2dButtonStateDebounced = 0;//Else the r2d button is released.
			}
			break;

		case VEHICLE_STATE_R2D_PROCEDURE_START: //Case 4 : The vehicle is entering the ready to drive procedure.
			al1OutputPinActualState = GPIO_PIN_SET;	//Pre - charge contact closes.
			al2OutputPinActualState = GPIO_PIN_SET;	//Main contact closes.
			frgOutputPinActualState = GPIO_PIN_RESET;//Front relay group opens.
			Bamocar_setSoftEnable(&myBamocar, false);//Inverter is still dead.

			if (!speakerHasPlayedForCurrentR2D) { //If the R2D sound hasn't been played by the speaker yet :
				speakerOutputPinActualState = GPIO_PIN_SET;	//Set speaker outpout pin to logical 1 , play the r2d sound.
				speakerOnTimeStart_ms = current_tick_ms;//The time the speaker starts is now.
				currentVehicleState = VEHICLE_STATE_R2D_PROCEDURE_SPEAKER; //Set Current Vehicle State to R2D_PROCEDURE_SPEAKER
			} else {
				currentVehicleState = VEHICLE_STATE_R2D_PROCEDURE_SPEAKER; // Ensure it moves on
			}
			break;

		case VEHICLE_STATE_R2D_PROCEDURE_SPEAKER:  //Case 5: R2D Procedure.
			al1OutputPinActualState = GPIO_PIN_SET; //Set pre charge pin to logical 1 , i.e. close the contact. 
			al2OutputPinActualState = GPIO_PIN_SET;	//Close the main contact to ensure full voltage.
			frgOutputPinActualState = GPIO_PIN_RESET; //Keep the FRG open to not power the inverter .
			speakerOutputPinActualState = GPIO_PIN_SET;	//Turn on the r2d sound.
			Bamocar_setSoftEnable(&myBamocar, false); //Keep the inverter dead , through CAN (soft disable)

			if ((current_tick_ms - speakerOnTimeStart_ms)
					>= R2D_SPEAKER_DURATION_MS) { // if the Sound has been playing for long enough: 
				speakerOutputPinActualState = GPIO_PIN_RESET; //Turn off the sound and clear the specific GPIO'S flag.
				speakerHasPlayedForCurrentR2D = true;	//Boolean Update
				r2d_button_physical_state = false; // Reset for next check

				if (!r2d_button_physical_state) { // R2D button must be released
					r2dButtonStateDebounced = 0; // Reset for next R2D attempt
					currentVehicleState = VEHICLE_STATE_READY_TO_DRIVE; //Change Current vehicle condition to R2D
					frgOutputPinActualState = GPIO_PIN_SET;	// Front relay group closes
					Bamocar_setSoftEnable(&myBamocar, true); //Inverter turns on through CAN.
				}
			}

			if ((r2dButtonStateDebounced && !r2d_button_physical_state //If the r2d button is pressed and now it is released and the r2d sound hasn't played yet
					&& !speakerHasPlayedForCurrentR2D)
					|| (!brakeEngagedPhysically //OR the brake hasn't been engaged physically AND the current Vehicle state is R2D_PROCEDURE_SPEAKER:
							&& currentVehicleState
									== VEHICLE_STATE_R2D_PROCEDURE_SPEAKER)) {
				r2dButtonStateDebounced = 0;  //The button is released
				currentVehicleState = VEHICLE_STATE_ACTIVATED; // Go back
				speakerOutputPinActualState = GPIO_PIN_RESET; //Sound set to logical 0 meaning it doesn't play.
				speakerHasPlayedForCurrentR2D = false; // Allow speaker next time
			}
			break;

		case VEHICLE_STATE_READY_TO_DRIVE: //Case 6: The Vehicle is in R2D State.
			al1OutputPinActualState = GPIO_PIN_SET;  //Close pre charge contact
			al2OutputPinActualState = GPIO_PIN_SET;  //Close main contact
			frgOutputPinActualState = GPIO_PIN_SET; //close front relay group contacts
			speakerOutputPinActualState = GPIO_PIN_RESET; //Do not play the R2D Sound. (Buzzer According to the rules)

			desired_torque_to_bamocar = MAP(apps1_percentage_filtered, 0, 100,
					0, 24314); //Parameters: Pedal % , Pedal % min , Pedal % max, min torque, max torque.
			Bamocar_setTorque(&myBamocar, desired_torque_to_bamocar); //Set the desired torque to the inverter through CAN

			// Exit R2D if AL button is pressed again
			if (al_button_physical_state) {
				if (!alButtonStateDebounced //if the activation logic button is released
				&& (current_tick_ms - lastALButtonDebounceTime //AND there has elapsed enough time so that another debounce check can be implemented:
				> DEBOUNCE_DELAY_MS)) {
					alButtonStateDebounced = 1;	//Then the button is pressed.
					lastALButtonDebounceTime = current_tick_ms;	//The time where the button was pressed is saved.
					currentVehicleState = VEHICLE_STATE_OFF;//Vehicle shuts off and exits ready to drive mode.
				}
			} else {
				alButtonStateDebounced = 0;	//Else the button is released.
			}
			break;

		case VEHICLE_STATE_APPS_FAULT://If the Accelerator Pedal Position Switch is faulty, or:
		case VEHICLE_STATE_BAMOCAR_FAULT://If the vehicle's inverter is faulty, or:
		case VEHICLE_STATE_BMS_FAULT://The vehicle's battery management system is faulty:
			frgOutputPinActualState = GPIO_PIN_RESET;	//Open FRG .
			al1OutputPinActualState = GPIO_PIN_RESET;//Open precharge contact.
			al2OutputPinActualState = GPIO_PIN_RESET;	//Open main contact. 
			speakerOutputPinActualState = GPIO_PIN_RESET;//No sound will play.
			Bamocar_setSoftEnable(&myBamocar, false);//Inverter is killed through CAN.
			Bamocar_setTorque(&myBamocar, 0.0f);//Inverter torque is set to 0 Nm through CAN.

			currentVehicleState = VEHICLE_STATE_OFF;	//Vehicle shuts off.

			// This state is latched. Requires system reset (e.g. power cycle).
			break;

		default:
			currentVehicleState = VEHICLE_STATE_OFF;//Vehicle is set to be shut off.
			break;
		}

		// --- 5. Update Physical Outputs (This Section actually drives the hardware pins to match what the state machine decided in 4)---
		HAL_GPIO_WritePin(AL_1_GPIO_Port, AL_1_Pin, al1OutputPinActualState); // Write value to pin 2 to drive the relay's pre-charge contact value to pin 1.
		HAL_GPIO_WritePin(AL_2_GPIO_Port, AL_2_Pin, al2OutputPinActualState);//Write value to pin 2 to drive the relay's main contact value to pin 2.
		HAL_GPIO_WritePin(FRG_CONTROL_GPIO_Port, FRG_CONTROL_Pin,
				frgOutputPinActualState);//Write 12V FRG Relay value to drive the FRG_CONTROL_PIN.
		HAL_GPIO_WritePin(SPEAKER_CONTROL_GPIO_Port, SPEAKER_CONTROL_Pin,
				speakerOutputPinActualState);//Write Speaker Control bit and drive the MCU's SPEAKER_CONTROL_Pin.

		// --- 6. Peripheral Control (Fan, Pump, Stepper) ---
		// Part A: Handle Manual Button Toggles (updates the state variables only)
		if (fan_button_physical_state) {  //When the fan button is pressed: 
			if (!fanButtonPressedRaw //And the specific button press hasn't been handled yet
			&& (current_tick_ms - lastFanButtonPressTime > DEBOUNCE_DELAY_MS)) { //AND there has elapsed enough time so that another debounce check can be implemented
				fanButtonPressedRaw = 1;	//Button is handled.
				lastFanButtonPressTime = current_tick_ms;//Last time the button was pressed was now.
				// Toggle the manual state for the fan
				fanOutputState =
						(fanOutputState == GPIO_PIN_SET) ?
								GPIO_PIN_RESET : GPIO_PIN_SET; //Turns on fan when off , or turns off fan when on
			}
		} else {
			fanButtonPressedRaw = 0;	//Else the press is not handled.
		}

		if (pump_button_physical_state) {	//If the pump button was pressed:
			if (!pumpButtonPressedRaw//AND the button pressing hasnt been handled
					&& (current_tick_ms - lastPumpButtonPressTime
							> DEBOUNCE_DELAY_MS)) { //And there has elapsed enough time so that another debounce check can be implemented
				pumpButtonPressedRaw = 1;	//Button is set as handled.
				lastPumpButtonPressTime = current_tick_ms;//Last button press time in ms is held.
				// Toggle the manual state for the pump
				pumpOutputState = (pumpOutputState == GPIO_PIN_SET) ?//Pump turns off when on , or on when off.
						GPIO_PIN_RESET : GPIO_PIN_SET;
			}
		} else {
			pumpButtonPressedRaw = 0;	//Else the button is not pressed.
		}

		// Part B: Check for Automatic Activation from Temperature
		motorTemp = Bamocar_getMotorTemp(&myBamocar); //Returns inverter temperature
		bool auto_cooling_required = (motorTemp > 70) || (controllerTemp > 70); //if motor or controller temperature is over 70 degrees, it is decided that automatic cooling is required.

		// Part C: Final Decision and Single Write to Hardware
		// The Fan is ON if auto cooling is required OR if it's manually turned on.
		if (auto_cooling_required || (fanOutputState == GPIO_PIN_SET)) { //If auto cooling is required or the fan is turned on by hand, the fan is on. Then the value is written to the FAN_CONTROL_Pin
			HAL_GPIO_WritePin(FAN_CONTROL_GPIO_Port, FAN_CONTROL_Pin,
					GPIO_PIN_SET);
		} else { //Else the fan is off and this specific state is written to the hardware pin named FAN_CONTROL_Pin.
			HAL_GPIO_WritePin(FAN_CONTROL_GPIO_Port, FAN_CONTROL_Pin,
					GPIO_PIN_RESET);
		}

		// --- 7. SPI DMA Communication --- About to be del. Na to valw mesa sto report, alla einai apokleistika gia ton eayto mou wste na dw pws douleyei. bebaia dhladh tha diagrafei to sygkekrimeno kommati kwdika apo to script.
		// Dhladh mas endiaferei to AIM, to BMS kai to Bamocar opwsdhpote. 
		if (spi_transfer_complete_flag) { // Check if the previous DMA transfer is finished
			spi_transfer_complete_flag = false; // Mark SPI as busy
			// 1. Populate Data Structure
			spi_tx_sequence_counter++;
			dataToSend.sequence_num = spi_tx_sequence_counter;
			dataToSend.data_length = sizeof(CombinedLogData_t);

			// --- Fill in your sensor data into the struct ---
			dataToSend.main_pedal1_filtered = apps1_percentage_filtered; //Log and store APPS sensor no.1 signal filter %
			dataToSend.main_pedal2_filtered = apps2_percentage_filtered;//Log and store APPS sensor no.2 signal filter %
			dataToSend.main_brake_engaged = brakeEngagedPhysically;	// Log and store physical brake pedal engagement (true if pressed)
			dataToSend.main_rtd_engaged = readyToDriveEngaged;// Log whether “Ready To Drive” status is active
			dataToSend.main_activation_engaged = activationEngaged;	// Log activation logic switch status (true if activation is engaged)
			dataToSend.main_fan_state =
					(fanOutputState == GPIO_PIN_SET) ? 1 : 0;// Log fan GPIO output state: 1 = on, 0 = off
			dataToSend.main_pump_state =
					(pumpOutputState == GPIO_PIN_SET) ? 1 : 0;// Log pump GPIO output state: 1 = on, 0 = off
			dataToSend.main_stepper_pos = currentStepperPosition;// Log current stepper motor position (in steps)
			dataToSend.main_rotary_val = rotarySwitchValue;	// Log raw rotary switch value 

			//CAN
			// --- BMS Data ---
			dataToSend.bms_pack_current = BMS_GetPackCurrent();	// Log the current draw of the battery pack 
			dataToSend.bms_pack_inst_voltage = BMS_GetPackInstVoltage();// Log the instantaneous pack voltage
			//Multiply the two above to compute Power and send to logger via CAN
			dataToSend.bms_pack_soc = BMS_GetPackSOC();	// Log the state‐of‐charge of the pack (percentage 0–100%)
			dataToSend.bms_pack_dcl = BMS_GetPackDCL();	// Log the pack’s current discharge limit (maximum allowable discharge current)
			dataToSend.bms_high_temp = BMS_GetHighTemp();// Log the highest cell temperature in the pack (°C)
			dataToSend.bms_low_temp = BMS_GetLowTemp();	// Log the lowest cell temperature in the pack (°C)
			dataToSend.bms_pack_summed_voltage = BMS_GetPackSummedVoltage();// Log the sum of all individual cell voltages (total pack voltage)
			dataToSend.bms_high_cell_voltage = BMS_GetHighCellVoltage();// Log the highest individual cell voltage within the pack (volts)
			dataToSend.bms_low_cell_voltage = BMS_GetLowCellVoltage();// Log the lowest individual cell voltage within the pack (volts)
			dataToSend.bms_max_pack_dcl = BMS_GetMaxPackDCL();// Log the maximum allowable discharge current limit for the pack (A)
			dataToSend.bms_max_pack_voltage = BMS_GetMaxPackVoltage();// Log the maximum allowable pack voltage threshold (V)

			//CAN
			// --- Bamocar Data --- I need to send this data to our Data Logger via CAN
			dataToSend.bamocar_rpm = Bamocar_getSpeed(&myBamocar); // Log and store motor controller speed in RPM
			dataToSend.bamocar_rpm_actual_raw = myBamocar.got.N_ACTUAL; // Raw actual RPM from controller register
			dataToSend.bamocar_n_max_raw = myBamocar.got.N_MAX; // Raw configured maximum RPM register
			dataToSend.bamocar_torque_actual = Bamocar_getTorque(&myBamocar); // Log actual torque output via API (e.g., Nm)
			dataToSend.bamocar_torque_raw = myBamocar.got.TORQUE; // Raw torque register value
			dataToSend.bamocar_current_actual = Bamocar_getCurrent(&myBamocar); // Log actual motor current via API (A)
			dataToSend.bamocar_i_actual_raw = myBamocar.got.I_ACTUAL; // Raw actual current register value
			dataToSend.bamocar_i_device_raw = myBamocar.got.I_DEVICE; // Raw device-level current register
			dataToSend.bamocar_i_200pc_raw = myBamocar.got.I_200PC; // Raw 200% current threshold register
			dataToSend.bamocar_motor_temp_raw = myBamocar.got.TEMP_MOTOR; // Raw motor winding temperature register
			dataToSend.bamocar_controller_temp_raw = myBamocar.got.TEMP_IGBT; // Raw IGBT (controller) temperature register
			dataToSend.bamocar_air_temp_raw = myBamocar.got.TEMP_AIR; // Raw ambient/air temperature register
			dataToSend.bamocar_dc_voltage_raw = myBamocar.got.V_DC_ACTUAL; // Raw DC bus voltage register
			dataToSend.bamocar_error_status_raw = myBamocar.got.ERROR_STATUS; // Raw error status bits register
			dataToSend.bamocar_status_raw = myBamocar.got.STATUS; // Raw controller status register
			dataToSend.bamocar_hard_enabled = Bamocar_getHardEnable(&myBamocar); // Boolean: hardware-enable flag from controller

			//Needed
			// --- AIM Data ---
			dataToSend.aim_g_sensor_x = AIM_GetGSensorX();//Log and store G - force in the X axis
			dataToSend.aim_g_sensor_y = AIM_GetGSensorY();//Log and store G - force in the Y axis
			dataToSend.aim_g_sensor_z = AIM_GetGSensorZ();//Log and store G - force in the Z axis
			dataToSend.aim_pitch_angle = AIM_GetPitchAngle();//Log and store sensor's reading for vehicle pitch angle. (Meaning whether the vehicle nose faces up or down)
			dataToSend.aim_yaw_moment = AIM_GetYawMoment();	//Log and store twisting torque. positive yaw swings the vehicle to the right , wherease negative yaw to the left.
			dataToSend.aim_gps_signal_valid = AIM_GetGpsValid();//Log and store boolean that states whether the vehicle can compute its location based on gps signals or not.
			dataToSend.aim_gps_latitude = AIM_GetGpsLatitude();	//Log and store GPS latitude
			dataToSend.aim_gps_longitude = AIM_GetGpsLongitude();//Log and store GPS longtitude
			dataToSend.aim_gps_speed = AIM_GetGpsSpeed();//Log and store GPS speed meaning the velocity over the earth's surface as computed by the receiver
			dataToSend.aim_steering_angle = AIM_GetSteeringAngle();	//Log and store vehicle steering angle as computed by the GPS.
			dataToSend.aim_brake_pressure = AIM_GetBrakePressure();	//Log and store vehicle brake pressure.
			dataToSend.aim_angleroll = AIM_GetAngleroll();//Log and store vehicle roll angle (nose to tail rotation)
			dataToSend.aim_distance = AIM_GetDistance();//Log and store vehicle travel distance
			dataToSend.aim_distance_lap = AIM_GetDistanceLap();	//Log and store vehicle distance traveled in the specific / current lap
			dataToSend.aim_duration = AIM_GetDuration();//Log and store time since vehicle started the lap
			dataToSend.aim_pos_left_back_sensor = AIM_GetPosLeftBack();	//Log and store left back pos etc.
			dataToSend.aim_pos_right_back_sensor = AIM_GetPosRightBack();
			dataToSend.aim_pos_left_front_sensor = AIM_GetPosLeftFront();
			dataToSend.aim_pos_right_front_sensor = AIM_GetPosRightFront();
			dataToSend.aim_pedal_position = AIM_GetPedalPosition();	//APPSreading
			dataToSend.aim_brake_position = AIM_GetBrakePosition();	//Brake pos reading
			dataToSend.aim_wheel_speed_rb = AIM_GetWheelSpeedRB();//Wheel Speed in each vehicle corner
			dataToSend.aim_wheel_speed_lb = AIM_GetWheelSpeedLB();
			dataToSend.aim_wheel_speed_rf = AIM_GetWheelSpeedRF();
			dataToSend.aim_wheel_speed_lf = AIM_GetWheelSpeedLF();
			dataToSend.aim_speed_velocity_rb = AIM_GetSpeedVelocityRB();//Vehicle right back side velocity
			dataToSend.aim_speed_velocity_lb = AIM_GetSpeedVelocityLB();//Vehicle left back side velocity
			dataToSend.aim_speed_velocity_rf = AIM_GetSpeedVelocityRF();//Vehicle right front side velocity
			dataToSend.aim_speed_velocity_lf = AIM_GetSpeedVelocityLF();//Vehicle left front side velocity
			dataToSend.aim_time = AIM_GetTime();

			// --- FUNC Data --- Telemetry
			// To be continued
			dataToSend.func_balance_brake_value = FUNC_GetBalanceBrakeValue();
			dataToSend.func_engine_mode = FUNC_GetEngineMode();

			// 2. Calculate Checksum
			dataToSend.checksum = calculateCRC16((uint8_t*) &dataToSend,
					sizeof(CombinedLogData_t));

			// 3. Start SPI Transmit/Receive using DMA
			if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) &dataToSend,
					spiTxBuffer, sizeof(CombinedLogData_t)) != HAL_OK) {
				spi_transfer_complete_flag = true;
				Error_Handler();
			}
		}

		//BMS_SendAllDataToCan2(CAN_HandleTypeDef *hcan) {

		//BMS_SendAllDataToCan2(&hcan2);

		Bamocar_SendConvertedDataToCan1(&hcan1);
		// --- AiM logger frames -> CAN2---
		LoggerCAN_Send_Performance_Data(&dataToSend);       //It doesnt do anything.Check the aim.h for more details
		AiM_CAN_Send_Temperature_Data(&dataToSend);         // 0x501: motor/IGBT temps (uses myBamocar internally)
		AiM_CAN_Send_BamocarCurrentsA(&myBamocar.got);      // 0x503
		AiM_CAN_Send_BamocarCurrentsB(&myBamocar.got);      // 0x504
		AiM_CAN_Send_BamocarFOC(&myBamocar.got);            // 0x505
		AiM_CAN_Send_BamocarVoltages(&myBamocar.got);       // 0x506
		LoggerCAN_Send_APPS(&dataToSend);					//0x507
		AiM_CAN_Send_Mapped_Torque_Value(&myBamocar.got);		//0X508


		HAL_Delay(10);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;//Synchronize the clock
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;	//Configure Data Resolution
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;//Configure continuous conversion mode
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef sFilterConfig;

	// Filter banks 0-13 are for CAN1, 14-27 for CAN2.
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	// Configure Filter Bank 0 for Bamocar and 3 BMS messages using 16-bit ID List Mode.
	// Ensure STD_RX_ID, BMS_ID_... are defined in their respective headers.
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = STD_TX_ID << 5;     // ID 1
	sFilterConfig.FilterIdLow = BMS_ID_VOLT << 5;    // ID 2
	sFilterConfig.FilterMaskIdHigh = BMS_ID_MAIN << 5;   // ID 3
	sFilterConfig.FilterMaskIdLow = BMS_ID_LIMITS << 5;  // ID 4
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	// Configure Filter Bank 1 for the remaining BMS message.
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterIdHigh = BMS_ID_TEMP << 5;     // ID 5
	sFilterConfig.FilterIdLow = 0;                     // Unused
	sFilterConfig.FilterMaskIdHigh = 0;                // Unused
	sFilterConfig.FilterMaskIdLow = 0;                 // Unused
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 9;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 17999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			FAN_CONTROL_Pin | FRG_CONTROL_Pin | SPEAKER_CONTROL_Pin | AL_2_Pin
					| AL_1_Pin | CONTROL_BJT_1_Pin | CONTROL_BJT_2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			PUMP_CONTROL_Pin | BRAKE_L_C_Pin | LED_Pin | DRS_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DIR_BB_Pin | STEP_BB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OE_LVL_SHFT_GPIO_Port, OE_LVL_SHFT_Pin, GPIO_PIN_RESET);//Configure GPIO Data for the SHIFT pins & ports

	/*Configure GPIO pins : PC13 FAN_BUTTON_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | FAN_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : FAN_CONTROL_Pin FRG_CONTROL_Pin SPEAKER_CONTROL_Pin AL_2_Pin
	 AL_1_Pin CONTROL_BJT_1_Pin CONTROL_BJT_2_Pin */
	GPIO_InitStruct.Pin = FAN_CONTROL_Pin | FRG_CONTROL_Pin
			| SPEAKER_CONTROL_Pin | AL_2_Pin | AL_1_Pin | CONTROL_BJT_1_Pin
			| CONTROL_BJT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PUMP_CONTROL_Pin BRAKE_L_C_Pin LED_Pin DRS_LED_Pin */
	GPIO_InitStruct.Pin = PUMP_CONTROL_Pin | BRAKE_L_C_Pin | LED_Pin
			| DRS_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PUMP_BUTTON_Pin A_L_BUTTON_Pin R2D_BUTTON_Pin BRAKE_PEDAL_Pin */
	GPIO_InitStruct.Pin = PUMP_BUTTON_Pin | A_L_BUTTON_Pin | R2D_BUTTON_Pin
			| BRAKE_PEDAL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_BB_Pin STEP_BB_Pin */
	GPIO_InitStruct.Pin = DIR_BB_Pin | STEP_BB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OE_LVL_SHFT_Pin */
	GPIO_InitStruct.Pin = OE_LVL_SHFT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OE_LVL_SHFT_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief This function centralizes checks for all system-level overrides.
 *        It checks for critical component faults and the shutdown circuit de-energizing.
 *        This should be called on every main loop cycle before the state machine runs.
 * @retval None
 */
void Check_For_System_Overrides(void) {
	// If we are already in a fault state, no other checks are needed.
	// The fault states are latched and require a power cycle to reset.
	if (currentVehicleState >= VEHICLE_STATE_APPS_FAULT) {
		return;
	}

	// --- 1. Check for Component Faults (Highest Priority) ---
	if (Check_APPS_Implausibility(apps1_percentage_filtered,
			apps2_percentage_filtered)) {
		currentVehicleState = VEHICLE_STATE_APPS_FAULT;
		return; // Fault found, exit early to prevent other checks this cycle
	}

	// Uncomment these checks when the underlying logic is confirmed to be working
	/*
	 if (bms_is_fault_active) {
	 currentVehicleState = VEHICLE_STATE_BMS_FAULT;
	 return;
	 }
	 if (Bamocar_checkStatus(&myBamocar) != 0) { // Assuming 0 is OK
	 currentVehicleState = VEHICLE_STATE_BAMOCAR_FAULT;
	 return;
	 }
	 */

	// --- 2. Check for Shutdown Circuit De-energization (Normal Shutdown) ---
	// First, "arm" the logic by setting a flag once the Tractive System is live (>60V).
	// This ensures we don't trigger a shutdown at initial power-on when voltage is 0.
	if (!high_voltage_was_active_flag
			&& (current_bamocar_dc_bus_voltage
					> SHUTDOWN_CIRCUIT_VOLTAGE_THRESHOLD)) {
		high_voltage_was_active_flag = true;
	}

	// If the logic is armed AND the voltage drops below the safe threshold, it means
	// the shutdown circuit has opened. This forces the car into the OFF state.
	if (high_voltage_was_active_flag
			&& (current_bamocar_dc_bus_voltage
					< SHUTDOWN_CIRCUIT_VOLTAGE_THRESHOLD)) {
		currentVehicleState = VEHICLE_STATE_OFF;
	}
}

// Fletcher-16 checksum calculation
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
	uint32_t sum1 = 0;
	uint32_t sum2 = 0;
	size_t checksum_offset = offsetof(CombinedLogData_t, checksum);

	// Process bytes before checksum field
	size_t i = 0;
	for (; i < checksum_offset; i++) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	// Skip checksum field (2 bytes) and process bytes after it
	i += sizeof(uint16_t);
	for (; i < length; i++) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return (sum2 << 8) | sum1;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
