/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
const float usPerDegree =11.11111;
const int usPerTick = 1;
const int initTime = 500;
int posX, posY;
int setDeg, setDegreeV;
int setPosX, setPosY;
int bounce;
int updated;

const int MOTOR1 =1;
const int MOTOR2 =2;
const int MOTOR3 =4;
const int MOTOR4 = 8;
uint8_t rxData[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setDegree(int motor,int degree){

	if(0 <= degree && degree <= 180 ){

		degree = (initTime + degree * usPerDegree)/usPerTick;

		if(motor & MOTOR1){
			htim1.Instance->CCR1 =degree;
		}
		if(motor & MOTOR2 ){
			htim1.Instance->CCR2 =degree;
		}
		if(motor & MOTOR3){
			htim1.Instance->CCR3 =degree;
		}
		if(motor & MOTOR4){
			htim1.Instance->CCR4 =degree;
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//   Prevent unused argument(s) compilation warning
  UNUSED(huart);
 /*  NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
*/



  if(rxData[0] == 10){ // set motor degree
	  setDegreeV = (rxData[2] << 8) | rxData[1];
	  setDeg =1;
	  updated = 1;
  }
  else if(rxData[0] == 50){ // current Position
	  posX = (rxData[2] << 8)  | rxData[1];
	  posY = (rxData[4] << 8) | rxData[3];
	  updated = 1;
  }else if(rxData[0] == 100){ // set Position
	  setPosX = (rxData[2] << 8)  | rxData[1];
	  setPosY = (rxData[4] << 8) | rxData[3];
	  updated = 1;
  }else if(rxData[0] == 150){ // bounce
	  bounce =1;
	  updated =1;
  }

}


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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_UART_Receive_DMA(&huart2, rxData, 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    posX = 0;
     posY = 0;
     setPosX =320;
     setPosY = 240;
     bounce =0;
     updated =0;

     int errorX, errorY;
     int prevX =0, prevY=0;
     int area =30;
     int pwmX, pwmY;

     int derivativeX, derivativeY;
     int integralX=0, integralY=0;

     // X Y  proportional
     float KpX = 0.06;
     float KpY = 0.06;

     // X Y derivative
     float KdX = 0.8;
     float KdY = 0.8;

     // X Y integral
     float KiX = 0.001;
     float KiY = 0.001;


     setDegree(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4, 5);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(updated){

	 	 			  if(setDeg){
	 	 				  setDegree(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4, setDegreeV);
	 	 				  setDeg =0;

	 	 				  prevX = 0;
	 	 				  prevY = 0;
	 	 				  integralX = 0;
	 	 				  integralY = 0;
	 	 			  }

	 	 			  else if(bounce){

	 	 				  for(int i=0;i<5;++i){
	 	 					  setDegree(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4,5);

	 	 					  HAL_Delay(200);

	 	 					  setDegree(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4,20);

	 	 					  HAL_Delay(200);
	 	 				  }

	 	 				  setDegree(MOTOR1 | MOTOR2 | MOTOR3 | MOTOR4,5);

	 	 				  bounce =0;

	 	 			  }
	 	 			  else{

	 	 				  errorX = setPosX - posX;
	 	 				  errorY = setPosY - posY;

	 	 				  derivativeX = errorX - prevX;
	 	 				  derivativeY = errorY - prevY;

	 	 				  integralX += errorX;
	 	 				  integralY += errorY;

	 	 				  pwmX = KpX * errorX + KdX * derivativeX + KiX * integralX;
	 	 				  pwmY = KpY * errorY + KdY * derivativeY + KiY * integralY;

	 	 				  if(50 < pwmX)
	 	 					  pwmX = 50;
	 	 				  else if(pwmX < -50)
	 	 					  pwmX = -50;

	 	 				  if(50 < pwmY)
	 	 					  pwmY = 50;
	 	 				  else if(pwmY < -50)
	 	 					  pwmY = -50;

	 	 				  if(errorX < -area || area < errorX){

	 	 					  if(0 < pwmX ){
	 	 						  setDegree(MOTOR1, 5+pwmX);
	 	 						  setDegree(MOTOR4, 5);
	 	 					  }else{
	 	 						  setDegree(MOTOR4, 5-pwmX);
	 	 						  setDegree(MOTOR1, 5);
	 	 					  }
	 	 				  }else{
	 	 					  setDegree(MOTOR1 | MOTOR4, 5);
	 	 					  integralX =0;
	 	 				  }

	 	 				  if(errorY < -area || area < errorY){
	 	 					  if(0 < pwmY ){
	 	 						  setDegree(MOTOR3, 5+pwmY);
	 	 						  setDegree(MOTOR2, 5);
	 	 					  }else{
	 	 						  setDegree(MOTOR2, 5-pwmY);
	 	 						  setDegree(MOTOR3, 5);
	 	 					  }
	 	 				  }
	 	 				  else{
	 	 					  setDegree(MOTOR2 | MOTOR3, 5);
	 	 					  integralY = 0;
	 	 				  }

	 	 				  prevX = errorX;
	 	 				  prevY = errorY;
	 	 			  }

	 	 			  updated =0;

	 	 		  }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
