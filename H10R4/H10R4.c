/*
 BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved

 File Name     : H10R4.c
 Description   : Source code for module H10R4 .


 Required MCU resources :

 >> UARTs 1,2,3,4,5 for module ports.
 >> ADC1 (Ch8 and Ch9)
 >>
 >>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "math.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Module exported parameters ------------------------------------------------*/
int H10R4_variant = 0;
float H10R4_x = 0;
float H10R4_y = 0;
bool H10R4_STREAM_TYPE = 0;

uint8_t startMeasurement = STOP_MEASUREMENT;
module_param_t modParam[NUM_MODULE_PARAMS] = { { .paramPtr = &H10R4_variant,
		.paramFormat = FMT_INT8, .paramName = "variant" }, { .paramPtr =
		&H10R4_x, .paramFormat = FMT_FLOAT, .paramName = "x" }, { .paramPtr =
		&H10R4_y, .paramFormat = FMT_FLOAT, .paramName = "y" }, { .paramPtr =
				&H10R4_STREAM_TYPE, .paramFormat = FMT_BOOL, .paramName = "streamType" } };

/* Private variables ---------------------------------------------------------*/
Stream_type_t type;
Stream_options_t option;
TimerHandle_t xTimerJoystick = NULL;
TaskHandle_t JoystickHandle = NULL;
volatile Joystick_state_t joystickState;
volatile uint32_t vector[2];
volatile uint8_t joystickMode;
volatile float x, y;
volatile float Right, Left, right, left;
int buf = 0;
int *ptrBuf = &buf;
int joystickMaxInterval = 0;
uint8_t joystickPort, joystickModule, v;
uint8_t stream_index = 0;
uint32_t joystickPeriod, joystickTimeout;
float Cbuf1 = 0, Cbuf2 = 0;
float *ptrCbuf1 = &Cbuf1, *ptrCbuf2 = &Cbuf2;
bool joystickVector;
bool stopB = 0;
bool variantB = 0;
bool buttonB = 0;

/* Private function prototypes -----------------------------------------------*/
TIM_HandleTypeDef htim3;
void JoystickTimerCallback(TimerHandle_t xTimerSwitch);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void JoystickTask(void *argument);
static uint32_t Adc_Calculation(int);
static void ADC_Select_CH8(void);
static void ADC_Select_CH9(void);
static void ADC_Deselect_CH8(void);
static void ADC_Deselect_CH9(void);
static int Get_Direction(void);
static void joystickStopMeasurement(void);
static void cartesianCoordinates(void);
static int calculateVariantValue(bool vector, int maxInterval);
static void CheckForEnterKey(void);
static Module_Status SendMeasurementResult(uint8_t request, uint8_t int_value,
		float fXValue, float fYValue, uint8_t module, uint8_t port,
		int *int_buffer, float *float_buffer1, float *float_buffer2);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE demoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);
static portBASE_TYPE joystickStreamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE variantModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE xModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE yModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE streamTypeModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE joystickStopCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
/*-----------------------------------------------------------*/

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition =
		{ (const int8_t*) "demo", /* The command string to type. */
				(const int8_t*) "demo:\r\nRun a demo program to test module functionality\r\n\r\n",
				demoCommand, 0 /* Zero parameter is expected. */
		};

/*-----------------------------------------------------------*/
/* CLI command structure : stream */
const CLI_Command_Definition_t joystickStreamCommandDefinition =
		{ (const int8_t*) "stream", /* The command string to type. */
				(const int8_t*) "stream:\r\nUse the stream command as follows:\n\r  >stream <Type_of_stream> <Where_to_stream> <period>(in ms) <Timeout>(in ms) <ParamA> <ParamB>\n\rType_of_streams:\n\r  -d\tDirection.\n\r  -c\tCarteisan. \n\rWhere_to_stream:\r\n  cli\t Outputs results on CLI.\r\n  raw\t Outputs RAW data.\r\n  port\t Outputs from Ports  ParamA:<P1..Px>  ParamB:<ModuleNum>.\n\r  variant\tStores values in a buffer accessed from the command: variant (Use with Direction stream)  ParamA:<Vector> (Y-axis=0, X-axis=1)  ParamB:<Max_Interval>.\n\r  buffer\tStores values in a buffer accessed from the commands: x and y (Use with Cartesian stream).\n\r\n\r",
				joystickStreamCommand, /* The function to run. */
				-1 /* Multiple parameters are expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : variant */
const CLI_Command_Definition_t variantModParamCommandDefinition =
		{ (const int8_t*) "variant", /* The command string to type. */
				(const int8_t*) "variant:\r\n Use the command: >stream -d variant <Period> <Timeout> <Vector> <Max_Interval>.\r\n Display the value of module parameter by typing: variant.\r\n\r\n",
				variantModParamCommand, /* The function to run. */
				0 /* No parameters are expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : x */
const CLI_Command_Definition_t xModParamCommandDefinition =
		{ (const int8_t*) "x", /* The command string to type. */
				(const int8_t*) "X-axis:\r\n Use the command: >stream -c buffer <Period> <Timeout>.\r\n Display the value of module parameter by typing: x.\r\n\r\n",
				xModParamCommand, /* The function to run. */
				0 /* No parameters are expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : y */
const CLI_Command_Definition_t yModParamCommandDefinition =
		{ (const int8_t*) "y", /* The command string to type. */
				(const int8_t*) "Y-axis:\r\n Use the command: >stream -c buffer <Period> <Timeout>.\r\n Display the value of module parameter by typing: y.\r\n\r\n",
				yModParamCommand, /* The function to run. */
				0 /* No parameters are expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : stream type */
const CLI_Command_Definition_t streamTypeModParamCommandDefinition =
		{ (const int8_t*) "streamType", /* The command string to type. */
				(const int8_t*) "Stream type:\r\n Display the value of module parameter: streamType\r\n\r\n",
				yModParamCommand, /* The function to run. */
				0 /* No parameters are expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : stop */
const CLI_Command_Definition_t joystickStopCommandDefinition = {
		(const int8_t*) "stop", /* The command string to type. */
		(const int8_t*) "stop:\r\n Stop continuous or timed measurement\r\n\r\n",
		joystickStopCommand, /* The function to run. */
		0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |				    	Private Functions	 						|							|
 -----------------------------------------------------------------------
 */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	__SYSCFG_CLK_ENABLE()
	;

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/*-----------------------------------------------------------*/
/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void) {
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 2, temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] = { 0 };

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID) {
		temp = (uint16_t) (N << 8) + myID;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, RO_START_ADDRESS, temp);
		FlashStatus = FLASH_WaitForLastOperation(
				(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t i = 1; i <= N; i++) {
			for (uint8_t j = 0; j <= MaxNumOfPorts; j++) {
				if (array[i - 1][0]) {
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
					RO_START_ADDRESS + add, array[i - 1][j]);
					add += 2;
					FlashStatus = FLASH_WaitForLastOperation(
							(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for (uint8_t s = 0; s < numOfRecordedSnippets; s++) {
		if (snippets[s].cond.conditionType) {
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy((uint8_t*) &snipBuffer[1], (uint8_t*) &snippets[s],
					sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for (uint8_t j = 0; j < (sizeof(snippet_t) / 2); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd,
						*(uint16_t*) &snipBuffer[j * 2]);
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for (uint8_t j = 0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, currentAdd,
						*(uint16_t*) (snippets[s].cmd + j * 2));
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 2;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void) {
	// Clear the array 
	memset(array, 0, sizeof(array));
	N = 1;
	myID = 0;

	return SaveToRO();
}

/* --- H10R4 module initialization ---
 */
void Module_Init(void) {

	/* GPIO init */
	MX_GPIO_Init();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	/* ADC init */
	MX_ADC_Init();

	/* Create a timeout timer */
	xTimerJoystick = xTimerCreate("JoystickTimer", pdMS_TO_TICKS(1000), pdFALSE,
			(void*) 1, JoystickTimerCallback);

	/* Create a Joystick task */
	xTaskCreate(JoystickTask, (const char* ) "JoystickTask",
			(2*configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &JoystickHandle);

}
/*-----------------------------------------------------------*/

/* --- H10R4 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src,
		uint8_t dst, uint8_t shift) {
	Module_Status result = H10R4_OK;
	  uint32_t period = 0;
	  uint32_t timeout = 0;
	  bool vector = 0;
	  int max = 0;

	switch (code) {

	case (CODE_H10R4_STREAM_TYPE):
		if (cMessage[port-1][shift] == 0)
			H10R4_STREAM_TYPE = 0;
		else
			H10R4_STREAM_TYPE = 1;
		break;


	case (CODE_H10R4_STOP):
		joystickMode = REQ_STOP;
		break;

	case (CODE_H10R4_STREAM_PORT):
			if(H10R4_STREAM_TYPE){
				type == cartesianStream;
			} else {
				type == directionStream;
			}
		period = ( (uint32_t) cMessage[port-1][2+shift] << 24 ) + ( (uint32_t) cMessage[port-1][3+shift] << 16 ) + ( (uint32_t) cMessage[port-1][4+shift] << 8 ) + cMessage[port-1][5+shift];
		timeout = ( (uint32_t) cMessage[port-1][6+shift] << 24 ) + ( (uint32_t) cMessage[port-1][7+shift] << 16 ) + ( (uint32_t) cMessage[port-1][8+shift] << 8 ) + cMessage[port-1][9+shift];
		Stream_To_Port(cMessage[port-1][shift], cMessage[port-1][1+shift], period, timeout);
		break;

	case (CODE_H10R4_STREAM_CLI):
			if(H10R4_STREAM_TYPE){
				type == cartesianStream;
			} else {
				type == directionStream;
			}
			period = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
			timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];
			Stream_To_CLI(period, timeout);
			break;

	case (CODE_H10R4_STREAM_RAW):
			if(H10R4_STREAM_TYPE){
				type == cartesianStream;
			} else {
				type == directionStream;
			}
			period = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
			timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];
			Stream_To_CLI_R(period, timeout);
			break;

	case (CODE_H10R4_STREAM_VARIANT):
		if(H10R4_STREAM_TYPE){
			type == cartesianStream;
		} else {
			type == directionStream;
		}
		period = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
		timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];
		buf = ( (uint32_t) cMessage[port-1][8+shift] << 24 ) + ( (uint32_t) cMessage[port-1][9+shift] << 16 ) + ( (uint32_t) cMessage[port-1][10+shift] << 8 ) + cMessage[port-1][11+shift];
		max = ( (uint32_t) cMessage[port-1][12+shift] << 24 ) + ( (uint32_t) cMessage[port-1][13+shift] << 16 ) + ( (uint32_t) cMessage[port-1][14+shift] << 8 ) + cMessage[port-1][15+shift];
		vector = ((bool) cMessage[port-1][16+shift]);
		calculateVariantValue(vector, max);
		Stream_To_Buffer(&buf, period, timeout);
		break;

	case (CODE_H10R4_STREAM_BUFFER):
		if(H10R4_STREAM_TYPE){
			type == cartesianStream;
		} else {
			type == directionStream;
		}
		period = ( (uint32_t) cMessage[port-1][shift] << 24 ) + ( (uint32_t) cMessage[port-1][1+shift] << 16 ) + ( (uint32_t) cMessage[port-1][2+shift] << 8 ) + cMessage[port-1][3+shift];
		timeout = ( (uint32_t) cMessage[port-1][4+shift] << 24 ) + ( (uint32_t) cMessage[port-1][5+shift] << 16 ) + ( (uint32_t) cMessage[port-1][6+shift] << 8 ) + cMessage[port-1][7+shift];
		Cbuf1 = ( (uint32_t) cMessage[port-1][8+shift] << 24 ) + ( (uint32_t) cMessage[port-1][9+shift] << 16 ) + ( (uint32_t) cMessage[port-1][10+shift] << 8 ) + cMessage[port-1][11+shift];
		Cbuf1 = ( (uint32_t) cMessage[port-1][12+shift] << 24 ) + ( (uint32_t) cMessage[port-1][13+shift] << 16 ) + ( (uint32_t) cMessage[port-1][14+shift] << 8 ) + cMessage[port-1][15+shift];
		Stream_To_Cbuffer(&Cbuf1, &Cbuf1, period, timeout);
		break;

	default:
		result = H10R4_ERR_UnknownMessage;
		break;
	}

	return result;
}
/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
 */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&demoCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&joystickStreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&joystickStopCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&variantModParamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&xModParamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&yModParamCommandDefinition);
}
/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART5)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;

	return 0;
}
/*-----------------------------------------------------------*/

/* --- Joystick timer callback ---*/
void JoystickTimerCallback(TimerHandle_t xTimerJoystick) {
	joystickStopMeasurement();

	uint32_t tid = 0;

	/* close DMA stream */
	tid = (uint32_t) pvTimerGetTimerID(xTimerJoystick);
	if (TIMERID_TIMEOUT_MEASUREMENT == tid) {
		startMeasurement = STOP_MEASUREMENT;
		joystickMode = REQ_IDLE;		// Stop the streaming task
	}

}
/*-----------------------------------------------------------*/

/* --- Joystick select button callback ---*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	buttonB = !buttonB;

}
/*-----------------------------------------------------------*/

/* --- Definition of Joystick Prime Task ---*/
static void JoystickTask(void *argument) {
	uint32_t t0 = 0;
	while (1) {
		switch (joystickMode) {

		case REQ_STREAM_CLI:
			t0 = HAL_GetTick();
			if (type == directionStream) {
				joystickState = Get_Direction();
				SendMeasurementResult(joystickMode, joystickState, 0, 0, 0, 0,
				NULL, NULL, NULL);
			} else if (type == cartesianStream) {
				cartesianCoordinates();
				SendMeasurementResult(joystickMode, 0, x, y, 0, 0,
				NULL,
				NULL, NULL);
			} else {
			}
			while (HAL_GetTick() - t0 < (joystickPeriod - 1) && !stopB) {
				taskYIELD();
			}
			break;

		case REQ_STREAM_RAW:
			t0 = HAL_GetTick();
			if (type == directionStream) {
				joystickState = Get_Direction();
				SendMeasurementResult(joystickMode, joystickState, 0, 0, 0, 0,
				NULL, NULL, NULL);
			} else if (type == cartesianStream) {
				cartesianCoordinates();
				SendMeasurementResult(joystickMode, 0, x, y, 0, 0,
				NULL,
				NULL, NULL);
			} else {
			}
			while (HAL_GetTick() - t0 < (joystickPeriod - 1) && !stopB) {
				taskYIELD();
			}
			break;

		case REQ_STREAM_PORT:
			if (type == directionStream) {
				t0 = HAL_GetTick();
				joystickState = Get_Direction();
				SendMeasurementResult(joystickMode, joystickState, 0, 0, 0,
						joystickPort,
						NULL, NULL, NULL);
			} else if (type == cartesianStream) {
				cartesianCoordinates();
				SendMeasurementResult(joystickMode, 0, x, y, 0, joystickPort,
				NULL, NULL, NULL);
			} else {
			}
			while (HAL_GetTick() - t0 < (joystickPeriod - 1) && !stopB) {
				taskYIELD();
			}
			break;

		case REQ_STREAM_BUFFER:
			if (type == directionStream) {
				t0 = HAL_GetTick();
				v = calculateVariantValue(joystickVector, joystickMaxInterval);
				SendMeasurementResult(joystickMode, v, 0, 0, joystickModule, 0,
						ptrBuf, NULL, NULL);
			} else if (type == cartesianStream) {
				cartesianCoordinates();
				SendMeasurementResult(joystickMode, 0, x, y, joystickModule, 0,
				NULL, ptrCbuf1, ptrCbuf2);
			} else {
			}
			while (HAL_GetTick() - t0 < (joystickPeriod - 1) && !stopB) {
				taskYIELD();
			}
			break;

		case REQ_STOP:
			Stop_Joystick();
			break;

		default:
			joystickMode = REQ_IDLE;
			buttonB = 0;
			break;
		}

		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

/* --- Send measurement results --- */
static Module_Status SendMeasurementResult(uint8_t request, uint8_t int_value,
		float fXValue, float fYValue, uint8_t module, uint8_t port,
		int *int_buffer, float *float_buffer1, float *float_buffer2) {

	Module_Status status = H10R4_OK;
	int8_t *message = (int8_t*) "";
	int8_t *pcOutputString;
	static int8_t *pcTMsg = "";
	static int8_t *pcTRawMsg = (int8_t*) "%s\r\n";
	static int8_t *pcDMsg =
			(int8_t*) "Cartesian Coordinates: X=%0.2f, Y=%0.2f\r\n";
	static int8_t *pcDRawMsg = (int8_t*) "%0.2f\t%0.2f\r\n";
	static const int8_t *pcOutTimeout = (int8_t*) "TIMEOUT\r\n";
	int8_t sendDirection;
	int sendTVariant;
	float sendCValue1, sendCValue2;
	float fXMessage = fXValue;
	float fYMessage = fYValue;

	static uint8_t temp[4];

	/* Get CLI output buffer */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	if (variantB) {
		pcTMsg = (int8_t*) "%d\r\n";
		sendTVariant = int_value;
		variantB = 0;
	} else {
		if (type == directionStream) {
			switch (int_value) {
			case UP:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Up";
				break;
			case DOWN:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Down";
				break;
			case RIGHT:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Right";
				break;
			case LEFT:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Left";
				break;
			case UP_RIGHT_CORNER:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Up_Right_Corner";
				break;
			case DOWN_RIGHT_CORNER:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Down_Right_Corner";
				break;
			case UP_LEFT_CORNER:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Up_Left_Corner";
				break;
			case DOWN_LEFT_CORNER:
				pcTMsg = (int8_t*) "We are going: %s !!\r\n";
				message = (char*) (int8_t*) "Down_Left_Corner";
				break;
			case BUTTON_CLICKED:
				pcTMsg = (int8_t*) "%s!!\r\n";
				message = (char*) (int8_t*) "Click";
				break;
			default:
				pcTMsg = (int8_t*) "We are not moving (%s) !!\r\n";
				message = (char*) (int8_t*) "Idle";
				break;
			}
		} else if (type == cartesianStream && joystickMode == REQ_STREAM_BUFFER) {
			sendCValue1 = fXValue;
			sendCValue2 = fYValue;
		}
	}

	// Send the value to appropriate outlet
	switch (joystickMode) {
	case REQ_STREAM_CLI:
		if (type == directionStream) {
			sprintf((char*) pcOutputString, (char*) pcTMsg, message);
		} else if (type == cartesianStream) {
			sprintf((char*) pcOutputString, (char*) pcDMsg, fXMessage,
					fYMessage);
		}
		writePxMutex(PcPort, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		CheckForEnterKey();
		break;

	case REQ_STREAM_RAW:
		if (type == directionStream) {
			sprintf((char*) pcOutputString, (char*) pcTRawMsg, message);
		} else if (type == cartesianStream) {
			sprintf((char*) pcOutputString, (char*) pcDRawMsg, fXMessage,
					fYMessage);
		}
		writePxMutex(PcPort, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		CheckForEnterKey();
		break;

	case REQ_STREAM_PORT:
		if (type == directionStream) {
			if (module == myID) {
				temp[0] = (int_value);
				writePxITMutex(port, (char*) &temp, sizeof(uint8_t), 10);
			} else {
				messageParams[0] = port;
				messageParams[1] = (int_value);
				SendMessageToModule(module, CODE_PORT_FORWARD,
						2);
			}
		} else if (type == cartesianStream) {

			if (module == myID) {

				// Send first value (X-axis)
				temp[0] = *((__IO uint8_t*) (&fXMessage) + 3);
				temp[1] = *((__IO uint8_t*) (&fXMessage) + 2);
				temp[2] = *((__IO uint8_t*) (&fXMessage) + 1);
				temp[3] = *((__IO uint8_t*) (&fXMessage) + 0);

				// Send second value (Y-axis)
				temp[4] = *((__IO uint8_t*) (&fYMessage) + 3);
				temp[5] = *((__IO uint8_t*) (&fYMessage) + 2);
				temp[6] = *((__IO uint8_t*) (&fYMessage) + 1);
				temp[7] = *((__IO uint8_t*) (&fYMessage) + 0);

				writePxITMutex(port, (char*) &temp[0], 8 * sizeof(uint8_t), 10);
			} else {
				messageParams[0] = port;
				messageParams[1] = *((__IO uint8_t*) (&fXMessage) + 3);
				messageParams[2] = *((__IO uint8_t*) (&fXMessage) + 2);
				messageParams[3] = *((__IO uint8_t*) (&fXMessage) + 1);
				messageParams[4] = *((__IO uint8_t*) (&fXMessage) + 0);

				messageParams[5] = *((__IO uint8_t*) (&fYMessage) + 3);
				messageParams[6] = *((__IO uint8_t*) (&fYMessage) + 2);
				messageParams[7] = *((__IO uint8_t*) (&fYMessage) + 1);
				messageParams[8] = *((__IO uint8_t*) (&fYMessage) + 0);
				SendMessageToModule(module, CODE_PORT_FORWARD,
						sizeof(uint32_t) + 1);
			}

		}
		break;

	case REQ_STREAM_BUFFER:
		if (type == directionStream) {
			memset(int_buffer, 0, sizeof(int));
			memcpy(int_buffer, &sendTVariant, sizeof(int));
		} else if (type == cartesianStream) {

			memset(float_buffer1, 0, sizeof(float));
			memcpy(float_buffer1, &sendCValue1, sizeof(float));
			memset(float_buffer2, 0, sizeof(float));
			memcpy(float_buffer2, &sendCValue2, sizeof(float));
		}
		break;

	default:
		break;
	}

	return (status);
}
/*-----------------------------------------------------------*/

/* --- Check for CLI stop key --- */
static void CheckForEnterKey(void) {
	stopB = 0;

	// Look for ENTER key to stop the stream
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf[PcPort - 1][chr] == '\r') {
			UARTRxBuf[PcPort - 1][chr] = 0;
			joystickMode = REQ_STOP;		// Stop the streaming task
			xTimerStop(xTimerJoystick, 0); // Stop any running timeout timer
			stopB = 1;
			break;
		}
	}
}
/*-----------------------------------------------------------*/

static void ADC_Select_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {

	}

}
/*-----------------------------------------------------------*/

static void ADC_Deselect_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_RANK_NONE;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {

	}
}
/*-----------------------------------------------------------*/

static void ADC_Select_CH9(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {

	}

}
/*-----------------------------------------------------------*/

static void ADC_Deselect_CH9(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_RANK_NONE;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {

	}
}
/* --- Calculate ADC value.
 */

static uint32_t Adc_Calculation(int selected) {

	switch (selected) {
	case 0:
		ADC_Select_CH8();
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 1000);
		vector[0] = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		ADC_Deselect_CH8();
		break;

	case 1:
		ADC_Select_CH9();
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 1000);
		vector[1] = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
		ADC_Deselect_CH9();
		break;

	default:
		break;
	}

	return *vector;

}
/*-----------------------------------------------------------*/

/* --- Get the Joystick direction.
 */
static int Get_Direction(void) {

	if (buttonB) {
		joystickState = BUTTON_CLICKED;
		buttonB = 0;
	} else {

		for (int i = 0; i < 2; ++i) {
			Adc_Calculation(i);

			if (vector[0] >= MIN_IDLE && vector[0] <= MAX_IDLE) {
				joystickState = IDLE;
			}
			if (vector[1] >= MIN_IDLE && vector[1] <= MAX_IDLE) {
				joystickState = IDLE;
			}
			if (vector[0] > MAX_Y) {
				joystickState = UP;
			}
			if (vector[0] < MIN_Y) {
				joystickState = DOWN;
			}
			if (vector[1] > MAX_X) {
				joystickState = LEFT;
			}
			if (vector[1] < MIN_X) {
				joystickState = RIGHT;
			}
			if (vector[0] > MAX_Y && vector[1] < MIN_X) {
				joystickState = UP_RIGHT_CORNER;
			}
			if (vector[0] < MIN_Y && vector[1] < MIN_X) {
				joystickState = DOWN_RIGHT_CORNER;
			}
			if (vector[0] > MAX_Y && vector[1] > MAX_X) {
				joystickState = UP_LEFT_CORNER;
			}
			if (vector[0] < MIN_Y && vector[1] > MAX_X) {
				joystickState = DOWN_LEFT_CORNER;
			}

		}

	}
	return joystickState;
}
/*-----------------------------------------------------------*/

/* --- Stop ADC Calculation ---*/
static void joystickStopMeasurement(void) {
	HAL_ADC_Stop(&hadc);
	ADC_Deselect_CH8();
	HAL_ADC_Stop(&hadc);
	ADC_Deselect_CH9();
}
/*-----------------------------------------------------------*/

/* --- Cartesian-Coordinates calculation ---*/
static void cartesianCoordinates(void) {

	for (int i = 0; i < 2; ++i) {
		Adc_Calculation(i);

		if (vector[0] >= MIN_IDLE && vector[0] <= MAX_IDLE) {
			y = 0.0;
		} else if (vector[0] > MAX_Y) {
			y = 100.0;
		} else {
			y = (vector[0] / 20.475) - 100;
		}

		if (vector[1] >= MIN_IDLE && vector[1] <= MAX_IDLE) {
			x = 0.0;
		} else if (vector[1] > MAX_X) {
			x = -100.0;
		} else {
			x = -(vector[1] / 20.475) + 100;
		}

	}

}
/*-----------------------------------------------------------*/

static int calculateVariantValue(bool vector, int maxInterval){

	variantB = 1;
	Get_Direction();


	if (buf <= maxInterval) {
		switch (vector) {
		case 0:
			if (joystickState == UP && buf < maxInterval
					&& joystickState != IDLE) {
				buf += 1;
				Delay_us(10);
			}
			if (joystickState == DOWN && buf > BASE_RANGE
					&& joystickState != IDLE) {
				buf -= 1;
				Delay_us(10);
			}
			break;
		case 1:
			if (joystickState == RIGHT && buf < maxInterval
					&& joystickState != IDLE) {
				buf += 1;
				Delay_us(10);
			}
			if (joystickState == LEFT && buf > BASE_RANGE
					&& joystickState != IDLE) {
				buf -= 1;
				Delay_us(10);
			}
			break;
		}
	} else {

		buf = maxInterval;
	}

	return buf;

}

/* -----------------------------------------------------------------------
 |					          APIs	 								|
 -----------------------------------------------------------------------
 */

/* --- Get Joystick Direction values.
 */
Module_Status Capture_joystick_state(int *result) {
	*result = Get_Direction();
	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- Get the Variant values.
 */
Module_Status Get_variant_value(bool *vector, int *maxInterval, int *buffer) {

	*buffer = calculateVariantValue(*vector, *maxInterval);
	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- Stop streaming Joystick values  --- */
Module_Status Stop_Joystick(void) {

	joystickMode = REQ_IDLE;
	startMeasurement = STOP_MEASUREMENT;
	xTimerStop(xTimerJoystick, 0);

	joystickStopMeasurement();

	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- stream values to CLI
 */
Module_Status Stream_To_CLI(uint32_t Period, uint32_t Timeout) {

	joystickPeriod = Period;
	joystickTimeout = Timeout;
	joystickMode = REQ_STREAM_CLI;

	if ((joystickTimeout > 0) && (joystickTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerJoystick = xTimerCreate("JoystickTimer",
				pdMS_TO_TICKS(joystickTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, JoystickTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerJoystick, portMAX_DELAY);
	}
	if (joystickTimeout > 0) {
		startMeasurement = START_MEASUREMENT;
	}

	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Raw values to CLI
 */
Module_Status Stream_To_CLI_R(uint32_t Period, uint32_t Timeout) {

	joystickPeriod = Period;
	joystickTimeout = Timeout;
	joystickMode = REQ_STREAM_RAW;

	if ((joystickTimeout > 0) && (joystickTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerJoystick = xTimerCreate("JoystickTimer",
				pdMS_TO_TICKS(joystickTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, JoystickTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerJoystick, portMAX_DELAY);
	}
	if (joystickTimeout > 0) {
		startMeasurement = START_MEASUREMENT;
	}
	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- Stream measurements continuously to a port --- */
Module_Status Stream_To_Port(uint8_t Port, uint8_t Module, uint32_t Period,
		uint32_t Timeout) {

	joystickPort = Port;
	joystickModule = Module;
	joystickPeriod = Period;
	joystickTimeout = Timeout;
	joystickMode = REQ_STREAM_PORT;

	if ((joystickTimeout > 0) && (joystickTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerJoystick = xTimerCreate("JoystickTimer",
				pdMS_TO_TICKS(joystickTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, JoystickTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerJoystick, portMAX_DELAY);
	}
	return (H10R4_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Variant values to buffer
 */
Module_Status Stream_To_Buffer(int *Buffer, uint32_t Period, uint32_t Timeout) {

	joystickPeriod = Period;
	joystickTimeout = Timeout;
	ptrBuf = Buffer;
	joystickMode = REQ_STREAM_BUFFER;

	if ((joystickTimeout > 0) && (joystickTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerJoystick = xTimerCreate("JoystickTimer",
				pdMS_TO_TICKS(joystickTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, JoystickTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerJoystick, portMAX_DELAY);
	}

	return (H10R4_OK);
}

/*-----------------------------------------------------------*/

/* --- stream Cartesian coordinates x and y to buffer
 */
Module_Status Stream_To_Cbuffer(float *Buffer1, float *Buffer2, uint32_t Period,
		uint32_t Timeout) {

	joystickPeriod = Period;
	joystickTimeout = Timeout;
	ptrCbuf1 = Buffer1;
	ptrCbuf2 = Buffer2;
	joystickMode = REQ_STREAM_BUFFER;

	if ((joystickTimeout > 0) && (joystickTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerJoystick = xTimerCreate("JoystickTimer",
				pdMS_TO_TICKS(joystickTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, JoystickTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerJoystick, portMAX_DELAY);
	}

	return (H10R4_OK);
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |					    	Commands		 						|									 	|
 -----------------------------------------------------------------------
 */

static portBASE_TYPE demoCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	int directionNum;
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	static int8_t *pcOKMessage = (int8_t*) "We are not moving (%s) !!\r\n";
	int8_t *directionVal = (int8_t*) "Idle";
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	directionNum = Get_Direction();

	switch (directionNum) {
	case UP:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Up";
		break;
	case DOWN:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Down";
		break;
	case RIGHT:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Right";
		break;
	case LEFT:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Left";
		break;
	case UP_RIGHT_CORNER:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Up_Right_Corner";
		break;
	case DOWN_RIGHT_CORNER:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Down_Right_Corner";
		break;
	case UP_LEFT_CORNER:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Up_Left_Corner";
		break;
	case DOWN_LEFT_CORNER:
		pcOKMessage = (int8_t*) "We are going: %s !!\r\n";
		directionVal = (char*) (int8_t*) "Down_Left_Corner";
		break;
	default:
		pcOKMessage = (int8_t*) "We are not moving (%s) !!\r\n";
		directionVal = (char*) (int8_t*) "Idle";
		break;
	}

	/* Respond to the command */
	sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, directionVal);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE joystickStreamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {

	static const int8_t *pcMessageModule =
			(int8_t*) "Streaming measurements to port P%d in module #%d\n\r";
	static const int8_t *pcMessageCLI =
			(int8_t*) "Streaming measurements to the CLI\n\n\r";
	static const int8_t *pcMessageError = (int8_t*) "Wrong parameter\r\n";
	static const int8_t *pcMessageWrongName = (int8_t*) "Wrong module name\r\n";
	static const int8_t *pcMessageStopMsg =
			(int8_t*) "Streaming stopped successfully\n\r";
	static const int8_t *pcMessageTbuffer =
			(char*) (int8_t*) "Streaming measurements to internal buffer. Access in the CLI using module parameters: variant\n\r";
	static const int8_t *pcMessageCbuffer =
			(char*) (int8_t*) "Streaming measurements to internal buffer. Access in the CLI using module parameters: x or y\n\r";
	int8_t *pcParameterString1;
	int8_t *pcParameterString2;
	int8_t *pcParameterString3;
	int8_t *pcParameterString4;
	int8_t *pcParameterString5;
	int8_t *pcParameterString6;
	portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0;
	portBASE_TYPE xParameterStringLength4 = 0;
	portBASE_TYPE xParameterStringLength5 = 0;
	portBASE_TYPE xParameterStringLength6 = 0;

	uint32_t Period = 0;
	uint32_t Timeout = 0;
	uint8_t Port = 0;
	uint8_t Module = 0;

	bool b;

	Module_Status result = H10R4_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string: Type */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	/* Obtain the 2nd parameter string: Period */
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);
	/* Obtain the 3rd parameter string: Timeout */
	pcParameterString3 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 3,
			&xParameterStringLength3);
	/* Obtain the 4rd parameter string: Options */
	pcParameterString4 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 4,
			&xParameterStringLength4);
	/* Obtain the 5rd parameter string: Module */
	pcParameterString5 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 5,
			&xParameterStringLength5);
	/* Obtain the 6rd parameter string: Module */
	pcParameterString6 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 6,
			&xParameterStringLength6);

	if (NULL != pcParameterString1
			&& !strncmp((const char*) pcParameterString1, "-d", 2)) {
		type = directionStream;
	} else if (NULL != pcParameterString1
			&& !strncmp((const char*) pcParameterString1, "-c", 2)) {
		type = cartesianStream;
	} else {
		result = H10R4_ERR_WrongParams;
	}

	if (NULL != pcParameterString3) {
		Period = atoi((char*) pcParameterString3);
	} else {
		result = H10R4_ERR_WrongParams;
	}
	if (NULL != pcParameterString4) {
		if (!strncmp((const char*) pcParameterString4, "inf", 3)) {
			Timeout = portMAX_DELAY;
		} else {
			Timeout = atoi((char*) pcParameterString4);
		}
	} else {
		result = H10R4_ERR_WrongParams;
	}

	if (NULL != pcParameterString2
			&& !strncmp((const char*) pcParameterString2, "cli", 3)) {
		option = cli;
	} else if (NULL != pcParameterString2
			&& !strncmp((const char*) pcParameterString2, "raw", 3)) {
		option = raw;
	} else if (NULL != pcParameterString2
			&& !strncmp((const char*) pcParameterString2, "variant", 7)) {
		option = variant;
	} else if (NULL != pcParameterString2
			&& !strncmp((const char*) pcParameterString2, "port", 4)) {
		option = port;
	} else if (NULL != pcParameterString2
			&& !strncmp((const char*) pcParameterString2, "buffer", 6)) {
		option = buffer;
	} else {

		result = H10R4_ERR_WrongParams;
	}

	switch (type) {

	case directionStream:
		switch (option) {
		case variant:
			if(result == H10R4_OK){
			joystickVector = atoi((char*) pcParameterString5);
			joystickMaxInterval = atoi((char*) pcParameterString6);
			strcpy((char*) pcWriteBuffer, (char*) pcMessageTbuffer);
			Stream_To_Buffer(&buf, Period, Timeout);
			}
			break;
		case raw:
			if(result == H10R4_OK){
			strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
			writePxMutex(PcPort, (char*) pcWriteBuffer,
					strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			Stream_To_CLI_R(Period, Timeout);

			/* Wait till the end of stream */
			while (startMeasurement != STOP_MEASUREMENT) {
				taskYIELD();
			}
			}
			break;
		case cli:
			if(result == H10R4_OK){
			strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
			writePxMutex(PcPort, (char*) pcWriteBuffer,
					strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			Stream_To_CLI(Period, Timeout);

			/* Wait till the end of stream */
			while (startMeasurement != STOP_MEASUREMENT) {
				taskYIELD();
			}
			}
			break;
		case port:
			if(result == H10R4_OK){
			Port = (uint8_t) atol((char*) pcParameterString5);
			Module = (uint8_t) atol((char*) pcParameterString6);
			if (result != (uint8_t) BOS_ERR_WrongName) {
				sprintf((char*) pcWriteBuffer, (char*) pcMessageModule, Port,
						Module);
				Stream_To_Port(Port, Module, Period, Timeout);

				// Return right away here as we don't want to block the CLI
				return pdFALSE;
			} else {
				strcpy((char*) pcWriteBuffer, (char*) pcMessageWrongName);
			}
			}
			break;
		default:
			result = H10R4_ERR_WrongParams;
			break;
		}
		break;

	case cartesianStream:
		switch (option) {
		case buffer:
			if(result == H10R4_OK){
			strcpy((char*) pcWriteBuffer, (char*) pcMessageCbuffer);
			Stream_To_Cbuffer(&Cbuf1, &Cbuf2, Period, Timeout);
			}
			break;
		case raw:
			if(result == H10R4_OK){
			strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
			writePxMutex(PcPort, (char*) pcWriteBuffer,
					strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			Stream_To_CLI_R(Period, Timeout);

			/* Wait till the end of stream */
			while (startMeasurement != STOP_MEASUREMENT) {
				taskYIELD();
			}
			}
			break;
		case cli:
			if(result == H10R4_OK){
			strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
			writePxMutex(PcPort, (char*) pcWriteBuffer,
					strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			Stream_To_CLI(Period, Timeout);

			/* Wait till the end of stream */
			while (startMeasurement != STOP_MEASUREMENT) {
				taskYIELD();
			}
			}
			break;
		case port:
			if(result == H10R4_OK){
			Port = (uint8_t) atol((char*) pcParameterString5);
			Module = (uint8_t) atol((char*) pcParameterString6);
			if (Module != (uint8_t) BOS_ERR_WrongName) {
				sprintf((char*) pcWriteBuffer, (char*) pcMessageModule, Port,
						Module);
				Stream_To_Port(Port, Module, Period, Timeout);

				// Return right away here as we don't want to block the CLI
				return pdFALSE;
			} else {
				strcpy((char*) pcWriteBuffer, (char*) pcMessageWrongName);
			}
			}
			break;
		default:
			result = H10R4_ERR_WrongParams;
			break;
		}
		break;
	}

	if (H10R4_ERR_WrongParams == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
		writePxMutex(PcPort, (char*) pcWriteBuffer,
						strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	}


	if (stopB) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageStopMsg);
		writePxMutex(PcPort, (char*) pcWriteBuffer,
				strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		memset((char*) pcWriteBuffer, 0, strlen((char*) pcWriteBuffer));
		stopB = 0;
	} else {
		/* clean terminal output */
		memset((char*) pcWriteBuffer, 0, strlen((char*) pcWriteBuffer));
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;

}
/*-----------------------------------------------------------*/

static portBASE_TYPE variantModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *Msg = (int8_t*) "%d\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf((char*) pcWriteBuffer, (char*) Msg, buf);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE xModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *Msg = (int8_t*) "%.2f\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf((char*) pcWriteBuffer, (char*) Msg, Cbuf1);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE yModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *Msg = (int8_t*) "%.2f\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf((char*) pcWriteBuffer, (char*) Msg, Cbuf2);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE streamTypeModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *Msg = (int8_t*) "%d\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf((char*) pcWriteBuffer, (char*) Msg, H10R4_STREAM_TYPE);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE joystickStopCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status result = H10R4_OK;
	static const int8_t *pcMessageOK =
			(int8_t*) "Streaming stopped successfully\r\n";
	static const int8_t *pcMessageError =
			(int8_t*) "Command failed! Please try again or reboot\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	result = Stop_Joystick();

	if (H10R4_OK == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageOK);
	} else {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
