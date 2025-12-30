/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include <stdio.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Data; //INCOMING DATA
char data[255] = {0}; // THE SENTENCE
volatile uint8_t idx = 0; // IDX IN SENTENCE
volatile uint8_t state = 0; // IN OR OUT OF MAIN
volatile uint16_t tp = 0; // TIME PERIOD
volatile uint16_t tp_cnt= 0; //TP COUNT
char err[] = "\r\nError: Unknown command Type HELP\r\n>";
char cmd[] = "\r\nCommands:\r\nSET PWM <10,20,..,100>   - Set LED brightness %\r\nSET BLINK <ms>           - Blink LED with period(10-2000) in ms\r\nSTATUS                   - Show system status\r\nIDLE                     - Set to idle mode\r\n>";
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	data[idx] = Data;
	idx++;
	if(Data == '\r' || Data == '\n') {
		state |= 1<<1;
		data[idx] = '\0';
	}
	else {
		HAL_UART_Transmit(&huart2,&Data,1,1);
		HAL_UART_Receive_IT(&huart2,&Data,1);
	}

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	++tp_cnt;
	if(htim->Instance == TIM7 && tp_cnt >= (tp/10)){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		tp_cnt = 0;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// This function acts as the "driver" for printf

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
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  volatile uint8_t dc = 0;


  char msg[] = "\r\n===== STM32 CLI Ready =====\r\nType HELP for commands\r\n>";
    char status[50] ;
    while (1)
    {
    	if(state == 0){
    	    		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),10);
    	    		state |= 1;
    	    	}
    	if(state & 1<<1){
    	    		//check for help
    				if( idx == 5 && strncmp(data,"HELP",4) == 0	){
    	    						HAL_UART_Transmit(&huart2,(uint8_t*)cmd,strlen(cmd),25);
    	    						snprintf(status,25, "\r\nMode : Idle\r\n>");
    	    				}
    	    				//check for PWM
    	    				else if(strncmp(data,"SET PWM ",8) == 0){
    	    					dc = (atoi((char*)data + 8))	;
    	    					if((dc % 10 !=0) || dc < 0 || dc > 100){
    	    						HAL_UART_Transmit(&huart2,(uint8_t*)"\r\nwrong values\r\n >",16,3);
    	    					}
    	    					else{
    	    						if(HAL_TIM_Base_GetState(&htim7) == HAL_TIM_STATE_BUSY){
    	    							HAL_TIM_Base_Stop(&htim7);
    	    						}
    	    						GPIO_InitTypeDef GPIO_InitStruct = {0};
    	    						GPIO_InitStruct.Pin = GPIO_PIN_5;
    	    						GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    	    							GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    	    							GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    	    							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    	    						HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    	    						__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(dc*999)/100);
    	    						char pwm_state[] = "\r\nDone!:)\r\n>";
    	    						snprintf(status,50, "\r\nMode      : PWM\r\nPWM Level : %d%%\r\n>",dc);
    	    						HAL_UART_Transmit(&huart2,(uint8_t*)pwm_state,strlen(pwm_state),1);
    	    					}
    	    				}
    	    				//blink
    	    				else if(!strncmp(data,"SET BLINK ",10)){
    	    					tp = atoi((char*)data + 10);
    	    					if(tp < 10 || tp >2000){
    	    					    	   HAL_UART_Transmit(&huart2,(uint8_t*)"\r\nwrong values\r\n >",16,3);
    	    					}else{
    	    						if(HAL_TIM_PWM_GetState(&htim2) == HAL_TIM_STATE_BUSY){
    	    							HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
    	    						}
    	    						 GPIO_InitTypeDef GPIO_InitStruct = {0};
    	    						 GPIO_InitStruct.Pin = GPIO_PIN_5;
    	    						 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	    						 GPIO_InitStruct.Pull = GPIO_NOPULL;
    	    						 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    	    						 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    	    						__HAL_TIM_SET_COUNTER(&htim7, 0);
    	    						tp_cnt = 0;
    	    						HAL_TIM_Base_Start_IT(&htim7);
    	    						snprintf(status,50, "\r\nMode        : Blink\r\nTime Period : %dms\r\n>",tp);
    	    						HAL_UART_Transmit(&huart2,(uint8_t*)"\r\nDONE\r\n>",9,2);
    	    					}
    	    				}
    	    				else if(!strncmp(data,"STATUS",6) && idx == 7){
    	    					HAL_UART_Transmit(&huart2,(uint8_t*)status,strlen(status),20);
    	    				}
    	    				else if(!strncmp(data,"IDLE",4) && idx == 5){
    	    					if(HAL_TIM_PWM_GetState(&htim2) == HAL_TIM_STATE_BUSY){
    	    						HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
    	    					}
    	    					if(HAL_TIM_Base_GetState(&htim7) == HAL_TIM_STATE_BUSY){
    	    						HAL_TIM_Base_Stop(&htim7);
    	    						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    	    					}
    	    					HAL_UART_Transmit(&huart2,(uint8_t*)"\r\nMade LED Off\r\n",18,2);
    	    					snprintf(status,25, "\r\nMode : Idle\r\n>");
    	    				}
    	    				else{
    	    						HAL_UART_Transmit(&huart2,(uint8_t*)err,strlen(err),5);
    	    				}
    	    				state &= ~(1<<1);
    	    				idx=0;

    	    	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	HAL_UART_Receive_IT(&huart2,&Data,1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 89;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
