/*
 BitzOS (BOS) V0.2.6 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H10R4_adc.c
 Description   : Source Code provides configuration of the ADC instances.

 */


/* Includes ------------------------------------------------------------------*/
#include "H10R4_adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init GPIO function */

void ADC_Channel_config(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hadc.Instance == ADC1) {
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**ADC GPIO Configuration
		 PB0     ------> ADC_IN8
		 PB1     ------> ADC_IN9
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	}
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
