/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
/* USER CODE BEGIN Private defines */
#define		SPEED_UP_PORT				GPIOA
#define		SPEED_DOWN_PORT			GPIOA


#define 	SPEED_UP_PIN				GPIO_PIN_8
#define		SPEED_DOWN_PIN			GPIO_PIN_7
#define		SPEED_UP_IDR				GPIO_IDR_IDR8
#define		SPEED_DOWN_IDR			GPIO_IDR_IDR7


#define		DIRECTION_PORT			GPIOA
#define		MOVE_UP_PORT				GPIOA
#define		MOVE_DOWN_PORT			GPIOA
#define		MOVE_LEFT_PORT			GPIOA
#define		MOVE_RIGHT_PORT			GPIOA

#define 	MOVE_UP_PIN					GPIO_PIN_15
#define 	MOVE_DOWN_PIN				GPIO_PIN_10
#define 	MOVE_LEFT_PIN				GPIO_PIN_11
#define 	MOVE_RIGHT_PIN			GPIO_PIN_12

#define 	MOVE_UP_IDR					GPIO_IDR_IDR15
#define 	MOVE_DOWN_IDR				GPIO_IDR_IDR10
#define 	MOVE_LEFT_IDR				GPIO_IDR_IDR11
#define 	MOVE_RIGHT_IDR			GPIO_IDR_IDR12

#define		MOVE_UP_BIT					0x01
#define		MOVE_DOWN_BIT				0x02
#define		MOVE_LEFT_BIT				0x04
#define		MOVE_RIGHT_BIT			0x08

#define		MIN_SERVO						0
#define		MAX_SERVO						0xFFFF

#define		SPEED_UP_BIT				0x01
#define		SPEED_DOWN_BIT			0x02

#define I2C_ADDRESS		0x2F
#define I2C_DUTYCYCLE	I2C_DUTYCYLE_2

#define	I2C_FRAME_LENGTH						3
#define	I2C_DATA_LENGTH							1
#define	I2C_COMMAND_BIT							0

#define MOVE_UP_COMMAND_START				0x01
#define	MOVE_UP_COMMAND_END					0x02

#define	MOVE_DOWN_COMMAND_START			0x03
#define MOVE_DOWN_COMMAND_END				0x04

#define	MOVE_LEFT_COMMAND_START			0x05
#define	MOVE_LEFT_COMMAND_END				0x06

#define	MOVE_RIGHT_COMMAND_START		0x07
#define	MOVE_RIGHT_COMMAND_END			0x08

#define	SPEED_UP_START							0x09
#define SPEED_UP_END								0x0A
#define	SPEED_DOWN_START						0x0B
#define	SPEED_DOWN_END							0x0C

#define	MASTER_CHECK_SPEED					0x1E

#define	LCD_DIRX_UP									0
#define	LCD_DIRX_DOWN								1
#define	LCD_DIRX_STOP								2


#define	LCD_DIRY_LEFT									0
#define	LCD_DIRY_RIGHT								1
#define	LCD_DIRY_STOP								2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
