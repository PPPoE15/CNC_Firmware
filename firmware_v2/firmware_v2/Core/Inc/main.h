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
void X_driver(void);
void Y_driver(void);
void Z_driver(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define endX_Pin GPIO_PIN_2
#define endX_GPIO_Port GPIOE
#define endY_Pin GPIO_PIN_3
#define endY_GPIO_Port GPIOE
#define endZ_Pin GPIO_PIN_4
#define endZ_GPIO_Port GPIOE
#define stepX_Pin GPIO_PIN_4
#define stepX_GPIO_Port GPIOA
#define stepY_Pin GPIO_PIN_5
#define stepY_GPIO_Port GPIOA
#define stepZ_Pin GPIO_PIN_6
#define stepZ_GPIO_Port GPIOA
#define dirX_Pin GPIO_PIN_7
#define dirX_GPIO_Port GPIOA
#define dirY_Pin GPIO_PIN_4
#define dirY_GPIO_Port GPIOC
#define dirZ_Pin GPIO_PIN_5
#define dirZ_GPIO_Port GPIOC
#define Enable_Pin GPIO_PIN_0
#define Enable_GPIO_Port GPIOB
#define Air_Pin GPIO_PIN_7
#define Air_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */