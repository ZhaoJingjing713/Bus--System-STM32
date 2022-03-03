/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SWITCH1_Pin GPIO_PIN_2
#define SWITCH1_GPIO_Port GPIOE
#define SWITCH2_Pin GPIO_PIN_3
#define SWITCH2_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define BG_Pin GPIO_PIN_2
#define BG_GPIO_Port GPIOC
#define Green_LED_Pin GPIO_PIN_0
#define Green_LED_GPIO_Port GPIOB
#define Red_LED_Pin GPIO_PIN_14
#define Red_LED_GPIO_Port GPIOB
#define myLCD_DC_Pin GPIO_PIN_15
#define myLCD_DC_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_11
#define RST_GPIO_Port GPIOD
#define HOT_CAMERA_TX_Pin GPIO_PIN_9
#define HOT_CAMERA_TX_GPIO_Port GPIOA
#define HOT_CAMERA_RX_Pin GPIO_PIN_10
#define HOT_CAMERA_RX_GPIO_Port GPIOA
#define CAMERA_PWR_DWN_Pin GPIO_PIN_11
#define CAMERA_PWR_DWN_GPIO_Port GPIOC
#define CAMERA_RESET_Pin GPIO_PIN_12
#define CAMERA_RESET_GPIO_Port GPIOC
#define WIFI_RST_Pin GPIO_PIN_8
#define WIFI_RST_GPIO_Port GPIOB
#define WIFI_CS_Pin GPIO_PIN_9
#define WIFI_CS_GPIO_Port GPIOB
#define Yellow_LED_Pin GPIO_PIN_1
#define Yellow_LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
