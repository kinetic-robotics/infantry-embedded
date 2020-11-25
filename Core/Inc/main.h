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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define IMU_HEAT_POWER_Pin GPIO_PIN_5
#define IMU_HEAT_POWER_GPIO_Port GPIOB
#define POWER_1_Pin GPIO_PIN_2
#define POWER_1_GPIO_Port GPIOH
#define FRICTION1_CONTROL_Pin GPIO_PIN_8
#define FRICTION1_CONTROL_GPIO_Port GPIOA
#define POWER_2_Pin GPIO_PIN_3
#define POWER_2_GPIO_Port GPIOH
#define POWER_3_Pin GPIO_PIN_4
#define POWER_3_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOG
#define POWER_4_Pin GPIO_PIN_5
#define POWER_4_GPIO_Port GPIOH
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOG
#define IMU_NSS_Pin GPIO_PIN_6
#define IMU_NSS_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_3
#define LED6_GPIO_Port GPIOG
#define LED7_Pin GPIO_PIN_2
#define LED7_GPIO_Port GPIOG
#define LED8_Pin GPIO_PIN_1
#define LED8_GPIO_Port GPIOG
#define FRICTION2_CONTROL_Pin GPIO_PIN_14
#define FRICTION2_CONTROL_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
