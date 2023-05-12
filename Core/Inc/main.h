/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_RST_Pin GPIO_PIN_13
#define GPS_RST_GPIO_Port GPIOC
#define ETH_SPDLED_Pin GPIO_PIN_0
#define ETH_SPDLED_GPIO_Port GPIOC
#define ADDR0_Pin GPIO_PIN_1
#define ADDR0_GPIO_Port GPIOC
#define ADDR1_Pin GPIO_PIN_2
#define ADDR1_GPIO_Port GPIOC
#define ADDR2_Pin GPIO_PIN_3
#define ADDR2_GPIO_Port GPIOC
#define ADDR3_Pin GPIO_PIN_1
#define ADDR3_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define ETH_CS_Pin GPIO_PIN_4
#define ETH_CS_GPIO_Port GPIOA
#define ETH_SCLK_Pin GPIO_PIN_5
#define ETH_SCLK_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_6
#define ETH_MISO_GPIO_Port GPIOA
#define ETH_MOSI_Pin GPIO_PIN_7
#define ETH_MOSI_GPIO_Port GPIOA
#define ETH_LINKLED_Pin GPIO_PIN_4
#define ETH_LINKLED_GPIO_Port GPIOC
#define ETH_DUPLED_Pin GPIO_PIN_5
#define ETH_DUPLED_GPIO_Port GPIOC
#define ETH_RST_Pin GPIO_PIN_0
#define ETH_RST_GPIO_Port GPIOB
#define ETH_INT_Pin GPIO_PIN_1
#define ETH_INT_GPIO_Port GPIOB
#define ETH_INT_EXTI_IRQn EXTI1_IRQn
#define ETH_ACTLED_Pin GPIO_PIN_10
#define ETH_ACTLED_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define MEM_RST_Pin GPIO_PIN_7
#define MEM_RST_GPIO_Port GPIOC
#define MEM_CS_Pin GPIO_PIN_8
#define MEM_CS_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_9
#define MEM_WP_GPIO_Port GPIOC
#define RF_CS_Pin GPIO_PIN_8
#define RF_CS_GPIO_Port GPIOA
#define RF_RST_Pin GPIO_PIN_9
#define RF_RST_GPIO_Port GPIOA
#define RF_DIO5_Pin GPIO_PIN_10
#define RF_DIO5_GPIO_Port GPIOA
#define RF_DIO4_Pin GPIO_PIN_11
#define RF_DIO4_GPIO_Port GPIOA
#define RF_DIO3_Pin GPIO_PIN_12
#define RF_DIO3_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_10
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_11
#define DEBUG_RX_GPIO_Port GPIOC
#define RF_DIO0_Pin GPIO_PIN_12
#define RF_DIO0_GPIO_Port GPIOC
#define RF_DIO1_Pin GPIO_PIN_2
#define RF_DIO1_GPIO_Port GPIOD
#define RF_DIO2_Pin GPIO_PIN_5
#define RF_DIO2_GPIO_Port GPIOB
#define GPS_SCL_Pin GPIO_PIN_6
#define GPS_SCL_GPIO_Port GPIOB
#define GPS_SDA_Pin GPIO_PIN_7
#define GPS_SDA_GPIO_Port GPIOB
#define GPS_HBEAT_Pin GPIO_PIN_8
#define GPS_HBEAT_GPIO_Port GPIOB
#define GPS_INT_Pin GPIO_PIN_9
#define GPS_INT_GPIO_Port GPIOB
#define GPS_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
