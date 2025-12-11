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
#include "stm32g4xx_hal.h"

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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_OK_Pin GPIO_PIN_13
#define Button_OK_GPIO_Port GPIOC
#define Button_OK_EXTI_IRQn EXTI15_10_IRQn
#define BUCKBOOST_LOAD_1_Pin GPIO_PIN_14
#define BUCKBOOST_LOAD_1_GPIO_Port GPIOC
#define BUCKBOOST_LOAD_2_Pin GPIO_PIN_15
#define BUCKBOOST_LOAD_2_GPIO_Port GPIOC
#define Button_R_L_IN_A_Pin GPIO_PIN_0
#define Button_R_L_IN_A_GPIO_Port GPIOF
#define Button_R_L_IN_B_Pin GPIO_PIN_1
#define Button_R_L_IN_B_GPIO_Port GPIOF
#define Button_R_L_SEL_0_Pin GPIO_PIN_0
#define Button_R_L_SEL_0_GPIO_Port GPIOC
#define Button_R_L_PWM_Pin GPIO_PIN_1
#define Button_R_L_PWM_GPIO_Port GPIOC
#define USBPD_VIN_Pin GPIO_PIN_2
#define USBPD_VIN_GPIO_Port GPIOC
#define BUCKBOOST_USBPD_EN_Pin GPIO_PIN_3
#define BUCKBOOST_USBPD_EN_GPIO_Port GPIOC
#define BUCK_GREEN_SENSE_Pin GPIO_PIN_0
#define BUCK_GREEN_SENSE_GPIO_Port GPIOA
#define BUCKBOOST_VIN_Pin GPIO_PIN_1
#define BUCKBOOST_VIN_GPIO_Port GPIOA
#define BUCKBOOST_I_IN_AVG_Pin GPIO_PIN_2
#define BUCKBOOST_I_IN_AVG_GPIO_Port GPIOA
#define BUCKBOOST_VOUT_Pin GPIO_PIN_3
#define BUCKBOOST_VOUT_GPIO_Port GPIOA
#define Button_LED_IN_B_Pin GPIO_PIN_4
#define Button_LED_IN_B_GPIO_Port GPIOA
#define BUCK_RED_SENSE_Pin GPIO_PIN_7
#define BUCK_RED_SENSE_GPIO_Port GPIOA
#define Button_LEFT_Pin GPIO_PIN_4
#define Button_LEFT_GPIO_Port GPIOC
#define Button_LEFT_EXTI_IRQn EXTI4_IRQn
#define Button_DOWN_Pin GPIO_PIN_5
#define Button_DOWN_GPIO_Port GPIOC
#define Button_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define BUCK_BLUE_SENSE_Pin GPIO_PIN_0
#define BUCK_BLUE_SENSE_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define Button_RIGHT_Pin GPIO_PIN_2
#define Button_RIGHT_GPIO_Port GPIOB
#define Button_RIGHT_EXTI_IRQn EXTI2_IRQn
#define Button_UP_Pin GPIO_PIN_10
#define Button_UP_GPIO_Port GPIOB
#define Button_UP_EXTI_IRQn EXTI15_10_IRQn
#define BUCKBOOST_I_IN_SENSE_Pin GPIO_PIN_11
#define BUCKBOOST_I_IN_SENSE_GPIO_Port GPIOB
#define BUCKBOOST_P1_DRIVE_Pin GPIO_PIN_12
#define BUCKBOOST_P1_DRIVE_GPIO_Port GPIOB
#define BUCKBOOST_N1_DRIVE_Pin GPIO_PIN_13
#define BUCKBOOST_N1_DRIVE_GPIO_Port GPIOB
#define Button_Deadzone_Pin GPIO_PIN_14
#define Button_Deadzone_GPIO_Port GPIOB
#define BUCKBOOST_P2_DRIVE_Pin GPIO_PIN_15
#define BUCKBOOST_P2_DRIVE_GPIO_Port GPIOB
#define BUCK_RED_DRIVE_Pin GPIO_PIN_6
#define BUCK_RED_DRIVE_GPIO_Port GPIOC
#define RC_TP4_Pin GPIO_PIN_7
#define RC_TP4_GPIO_Port GPIOC
#define BUCK_GREEN_DRIVE_Pin GPIO_PIN_8
#define BUCK_GREEN_DRIVE_GPIO_Port GPIOC
#define BUCK_BLUE_DRIVE_Pin GPIO_PIN_8
#define BUCK_BLUE_DRIVE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_15
#define LD2_GPIO_Port GPIOA
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOC
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOC
#define USBPD_1A_PROTECT_Pin GPIO_PIN_12
#define USBPD_1A_PROTECT_GPIO_Port GPIOC
#define USBPD_550mA_PROTECT_Pin GPIO_PIN_2
#define USBPD_550mA_PROTECT_GPIO_Port GPIOD
#define Button_U_D_PWM_Pin GPIO_PIN_5
#define Button_U_D_PWM_GPIO_Port GPIOB
#define Button_U_D_SEL_0_Pin GPIO_PIN_7
#define Button_U_D_SEL_0_GPIO_Port GPIOB
#define Button_U_D_IN_B_Pin GPIO_PIN_8
#define Button_U_D_IN_B_GPIO_Port GPIOB
#define Button_U_D_IN_A_Pin GPIO_PIN_9
#define Button_U_D_IN_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
