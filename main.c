/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" 
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//char buf[40];
volatile	uint8_t	aTxBuffer[I2C_FRAME_LENGTH];
volatile	uint8_t	aRxBuffer[I2C_FRAME_LENGTH];
volatile	uint8_t	time_count = 0;//for timer3
volatile 	uint8_t	rec_status	=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Send_Command(uint8_t);
void ReceiveData(void);
void Update_LCD_Command(void);
void Update_LCD_Speed(void);
void Send_LCD_DirX(uint8_t);
void Send_LCD_DirY(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_goto_XY(1,0);
	//lcd_send_string("Wn1511-Team3");
	//lcd_goto_XY(2,0);
	lcd_send_string("Dy:stop  Team3");
	HAL_Delay(40);
	lcd_goto_XY(2,0);
	lcd_send_string("Dx:stop  Sp:000%");
	//HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == MOVE_UP_PIN)
	{
		if(DIRECTION_PORT->IDR & MOVE_UP_IDR)// when not press MOVE_UP button
		{
			Send_Command(MOVE_UP_COMMAND_END);
			Send_LCD_DirY(LCD_DIRX_STOP);
		}
		else{//when press MOVE_UP button
			Send_Command(MOVE_UP_COMMAND_START);
			Send_LCD_DirY(LCD_DIRX_UP);
		}
	}
	
	if(GPIO_Pin == MOVE_DOWN_PIN)
	{
		if(DIRECTION_PORT->IDR & MOVE_DOWN_IDR) // when not press MOVE_DOWN	button
		{
			Send_Command(MOVE_DOWN_COMMAND_END);
			Send_LCD_DirY(LCD_DIRX_STOP);
		}
		else{//when press MOVE_DOWN button
			Send_Command(MOVE_DOWN_COMMAND_START);
			Send_LCD_DirY(LCD_DIRX_DOWN);
		}	
	}
	
	if(GPIO_Pin == MOVE_LEFT_PIN)
	{
		if(DIRECTION_PORT->IDR & MOVE_LEFT_IDR) // when not press MOVE_LEFT button
		{
			Send_Command(MOVE_LEFT_COMMAND_END);
			Send_LCD_DirX(LCD_DIRY_STOP);
		}
		else{//when press MOVE_LEFT button
			Send_Command(MOVE_LEFT_COMMAND_START);
			Send_LCD_DirX(LCD_DIRY_LEFT);
		}
	}
	
	if(GPIO_Pin == MOVE_RIGHT_PIN)
	{
		if(DIRECTION_PORT->IDR & MOVE_RIGHT_IDR)// when not press MOVE_RIGHT button
		{
			Send_Command(MOVE_RIGHT_COMMAND_END);
			Send_LCD_DirX(LCD_DIRY_STOP);
		}
		else{//when press MOVE_RIGHT button
			Send_Command(MOVE_RIGHT_COMMAND_START);
			Send_LCD_DirX(LCD_DIRY_RIGHT);
			
		}
	}
	
	if(GPIO_Pin == SPEED_DOWN_PIN)
	{
		if(SPEED_DOWN_PORT->IDR & SPEED_DOWN_IDR)// when not press SPEED_DOWN button
		{
			Send_Command(SPEED_DOWN_END);	
			HAL_TIM_Base_Stop_IT(&htim2);			
		}
		else 
		{//when press SPEED_DOWN button
			Send_Command(SPEED_DOWN_START);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
	
	if(GPIO_Pin == SPEED_UP_PIN)
	{
		if(SPEED_UP_PORT->IDR & SPEED_UP_IDR)// when not press SPEED_UP button
		{
			Send_Command(SPEED_UP_END);
			HAL_TIM_Base_Stop_IT(&htim2);
		//return;
		}
		else 
		{//when press SPEED_UP button
			Send_Command(SPEED_UP_START);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
}
void Send_Command(uint8_t command)
{
	uint8_t buf[I2C_FRAME_LENGTH] = {0} ;
	buf[I2C_COMMAND_BIT] = command;
	/*do
	{
		if(HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_ADDRESS, buf , 1) != HAL_OK)
		{
			Error_Handler();
		}
		
		//while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	}
	while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);*/
	while(HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS,(uint8_t*) buf,  I2C_FRAME_LENGTH, 1000) != HAL_OK)
  {
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}
}
void Update_LCD_Command(void)
{
	char *buf;
	
	
	
	lcd_send_string(buf);
	
	
}
void Send_LCD_DirY(uint8_t command)
{
	lcd_goto_XY(1,3);
	if(command == LCD_DIRX_DOWN)
	{
		lcd_send_string("down");
	}
	else if(command == LCD_DIRX_UP)
	{
		lcd_send_string("up  ");
	}
	else{
		lcd_send_string("stop");
	}
}
void Send_LCD_DirX(uint8_t command)
{
	lcd_goto_XY(2,3);
	if(command == LCD_DIRY_LEFT)
	{
		lcd_send_string("left ");
	}
	else if(command == LCD_DIRY_RIGHT)
	{
		lcd_send_string("right");
	}
	else{
		
		lcd_send_string("stop ");
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
	{
		Send_Command(MASTER_CHECK_SPEED);
		ReceiveData();
		Update_LCD_Speed();
	}
	else{
		if(time_count < 200)
		{
			time_count++;
		}
		else{
			lcd_goto_XY(1, 15);
			if(rec_status & 0x01)
				lcd_send_string("*");
			else
				lcd_send_string(" ");
			rec_status^=0x01;
			time_count=0;
		}
	}
}
void ReceiveData(void)
{
	while(HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, (uint8_t*)aRxBuffer, I2C_FRAME_LENGTH, 1000) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			Error_Handler();
	}		
}

void Update_LCD_Speed(void)
{
	uint8_t tempBuf[I2C_DATA_LENGTH] = {0};
	for(int i = 0; i <= 3; i++)
		tempBuf[i] = aRxBuffer[i+1];
	if(strcmp((char*)tempBuf, "100") == 0 )
		lcd_goto_XY(2,12);
	else 
	{
		lcd_goto_XY(2,12);
		lcd_send_string("0");
		lcd_goto_XY(2,13);
	}
	lcd_send_string((char*)tempBuf);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		for(int i = 0 ; i < 400000; i++);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
