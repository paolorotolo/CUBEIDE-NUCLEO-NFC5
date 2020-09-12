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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define IRQ_3911_Pin GPIO_PIN_0
#define IRQ_3911_GPIO_Port GPIOA
#define IRQ_3911_EXTI_IRQn EXTI0_IRQn
#define LED_F_Pin GPIO_PIN_1
#define LED_F_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOA
#define LED_A_Pin GPIO_PIN_0
#define LED_A_GPIO_Port GPIOB
#define LED_FIELD_Pin GPIO_PIN_8
#define LED_FIELD_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_V_Pin GPIO_PIN_4
#define LED_V_GPIO_Port GPIOB
#define LED_AP2P_Pin GPIO_PIN_5
#define LED_AP2P_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB
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
#define IRQ_MCU_Pin GPIO_PIN_0
#define IRQ_MCU_GPIO_Port GPIOA
#define IRQ_MCU_EXTI_IRQn EXTI0_IRQn
#define MCU_LED1_Pin GPIO_PIN_1
#define MCU_LED1_GPIO_Port GPIOA
#define MCU_LED2_Pin GPIO_PIN_4
#define MCU_LED2_GPIO_Port GPIOA
#define MCU_LED3_Pin GPIO_PIN_0
#define MCU_LED3_GPIO_Port GPIOB
#define MCU_LED6_Pin GPIO_PIN_8
#define MCU_LED6_GPIO_Port GPIOA
#define MCU_LED4_Pin GPIO_PIN_4
#define MCU_LED4_GPIO_Port GPIOB
#define MCU_LED5_Pin GPIO_PIN_5
#define MCU_LED5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
