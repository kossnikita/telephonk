/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_utils.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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
  void Send_Dial(uint8_t *num, uint8_t len);
  void Enable_Ring(void);
  void Down_Phone(void);
  void Up_Phone(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _GSM_KEY_PIN_Pin GPIO_PIN_15
#define _GSM_KEY_PIN_GPIO_Port GPIOC
#define IMPS_Pin GPIO_PIN_0
#define IMPS_GPIO_Port GPIOA
#define RING_N_Pin GPIO_PIN_7
#define RING_N_GPIO_Port GPIOA
#define OPEN_Pin GPIO_PIN_14
#define OPEN_GPIO_Port GPIOB
#define OPEN_EXTI_IRQn EXTI15_10_IRQn
#define DIAL_Pin GPIO_PIN_15
#define DIAL_GPIO_Port GPIOB
#define RING_P_Pin GPIO_PIN_8
#define RING_P_GPIO_Port GPIOA
#define GSM_TX_Pin GPIO_PIN_9
#define GSM_TX_GPIO_Port GPIOA
#define GSM_RX_Pin GPIO_PIN_10
#define GSM_RX_GPIO_Port GPIOA

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
