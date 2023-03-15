/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : H10R4.h
 Description   : Header file for module H10R4
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H10R4_H
#define H10R4_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H10R4_MemoryMap.h"
#include "H10R4_uart.h"
#include "H10R4_gpio.h"
#include "H10R4_dma.h"
#include "H10R4_adc.h"
#include "H10R4_inputs.h"
#include "H10R4_eeprom.h"
/* Exported definitions -------------------------------------------------------*/
#ifdef H10R4
#define	modulePN		_H10R4
#endif

/* Port-related definitions */
#define	NumOfPorts			5
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart5 1
#define _Usart6 1

/* Port-UART mapping */
#define P1uart &huart5
#define P2uart &huart2	
#define P3uart &huart6	
#define P4uart &huart3
#define P5uart &huart1

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT		GPIOB
#define	USART5_RX_PORT		GPIOB
#define	USART5_AF			GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF5_USART6

/* Module-specific Definitions */
#define NUM_MODULE_PARAMS			  4
#define MIN_X    					  1850
#define MAX_X						  2150
#define MIN_Y						  1850
#define MAX_Y						  2200
#define MIN_IDLE					  1900
#define MAX_IDLE					  2150
#define VERTICAL 					  1
#define HORIZENTAL					  !VERTICAL
#define BASE_RANGE					  0

#define JOYSTICK_DEFAULT_MAX_LOOP     2000

#define STOP_MEASUREMENT      		  0
#define START_MEASUREMENT     		  1

#define REQ_IDLE               		  0
#define REQ_STREAM_VARIANT            1
#define REQ_STREAM_RAW   			  2
#define REQ_STREAM_PORT		      	  3
#define REQ_STREAM_BUFFER         	  4
#define REQ_STREAM_CLI				  5
#define REQ_STREAM_DRAW   			  6
#define REQ_STREAM_DPORT		      7
#define REQ_STREAM_DBUFFER         	  8
#define REQ_STREAM_DCLI				  9
#define REQ_TIMEOUT             	  10
#define REQ_MEASUREMENT_READY         11
#define REQ_TIMEOUT_CLI				  12
#define REQ_TIMEOUT_VERBOSE_CLI		  13
#define REQ_TIMEOUT_BUFFER			  14
#define REQ_STOP					  15

#define TIMERID_TIMEOUT_MEASUREMENT   0xFF

/* Macros define Joystick running mode */
#define JOYSTICK_MODE_SINGLE            0x00
#define JOYSTICK_MODE_CONTINUOUS        0x01
#define JOYSTICK_MODE_CONTINUOUS_TIMED  0x02

/* Module EEPROM Variables */

/* Module_Status Type Definition */
typedef enum {
	IDLE,
	RIGHT,
	LEFT,
	UP,
	DOWN,
	UP_RIGHT_CORNER,
	DOWN_RIGHT_CORNER,
	UP_LEFT_CORNER,
	DOWN_LEFT_CORNER,
	BUTTON_CLICKED
} Joystick_state_t;

typedef enum {
	H10R4_OK = 0,
	H10R4_ERR_UnknownMessage = 1,
	H10R4_ERR_Wrong_Value = 2,
	H10R4_ERR_Timeout,
	H10R4_ERR_WrongParams,
	H10R4_STOPED,
	H10R4_ERROR = 255
} Module_Status;

typedef enum {
	directionStream, cartesianStream
} Stream_type_t;

typedef enum {
	variant, raw, cli, port, buffer, valuefixed,

} Stream_options_t;

/* Indicator LED */
#ifdef H10R4
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11
#endif


extern Stream_type_t type;
extern volatile uint8_t joystickMode;

/* Export GPIO variables */
extern void MX_GPIO_Init(void);

/* Export ADC variables */
extern void MX_ADC_Init(void);

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

extern TIM_HandleTypeDef htim3;

/* -----------------------------------------------------------------------
 |		        		    	APIs	 							|									 	|
 -----------------------------------------------------------------------
 */

extern Module_Status Capture_joystick_state(int *result);
extern Module_Status Capture_joystick_state4(int *result4);
extern Module_Status Get_variant_value(bool *vector, int *maxInterval, int *bufferx, int *buffery, int buffer_value_lock);
extern Module_Status Stop_Joystick(void);
extern Module_Status Stream_To_Buffer(int *Bufferx, int *Buffery, uint32_t Period,
		uint32_t Timeout);
extern Module_Status Stream_To_Cbuffer(float *Buffer1, float *Buffer2,
		uint32_t Period, uint32_t Timeout);
extern Module_Status Stream_To_Buffer_value_fixed(int *value_fixedx, int *value_fixedy, uint32_t Period,
		uint32_t Timeout);
extern Module_Status Stream_To_CLI(uint32_t Period, uint32_t Timeout);
extern Module_Status Stream_To_CLI_R(uint32_t Period, uint32_t Timeout);
extern Module_Status Stream_To_Port(uint8_t Port, uint8_t Module,
		uint32_t Period, uint32_t Timeout);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
/* -----------------------------------------------------------------------
 |			    			 Commands								|								 	|
 -----------------------------------------------------------------------
 */

#endif /* H10R4_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
