/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_TxHeaderTypeDef TxHeader;
CAN_FilterTypeDef filter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t TxMailbox;
uint8_t TxData[8];
uint32_t id;
uint32_t dlc;
uint8_t data[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxData)==HAL_OK){
		id  = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;
		dlc = RxHeader.DLC;
		for(uint8_t i = 0 ; i < 8 ; i ++ ){
			data[i] = RxData[i];
		}
	}
}
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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  uint32_t fid = 0x200;               // 0b 010 0000 0000 (32ビットだけど先頭が0なので11ビットとして扱える的な).
  uint32_t fmask = 0x7F0;             // 0b 111 1111 0000
  filter.FilterIdHigh = fid << 5;     // 0b 0100 0000 0000 0000
  filter.FilterIdLow = 0;             // 0b 0000 0000 0000 0000
  filter.FilterMaskIdHigh = fmask<<5; // 0b 1111 1110 0000 0000
  filter.FilterMaskIdLow = 0;         // 0b 0000 0000 0000 0000

  filter.FilterIdHigh = 0b0100000000000000;
  filter.FilterMaskIdHigh = 0b1111111110000000;

  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.SlaveStartFilterBank = 14;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan,&filter);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  setbuf(stdout, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(0<HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		  TxHeader.StdId = 0x401;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;
		  TxData[0] = 0x1;
		  TxData[1] = 0x1;
		  TxData[2] = 0x1;
		  TxData[3] = 0x1;
		  TxData[4] = 0x1;
		  TxData[5] = 0x1;
		  TxData[6] = 0x1;
		  TxData[7] = 0x1;
		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	  }

	  if(0<HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		  TxHeader.StdId = 0x202;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;
		  TxData[0] = 0x2;
		  TxData[1] = 0x2;
		  TxData[2] = 0x2;
		  TxData[3] = 0x2;
		  TxData[4] = 0x2;
		  TxData[5] = 0x2;
		  TxData[6] = 0x2;
		  TxData[7] = 0x2;
		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	  	  }

	  if(0<HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		  TxHeader.StdId = 0x203;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;
		  TxData[0] = 0x3;
		  TxData[1] = 0x3;
		  TxData[2] = 0x3;
		  TxData[3] = 0x3;
		  TxData[4] = 0x3;
		  TxData[5] = 0x3;
		  TxData[6] = 0x3;
		  TxData[7] = 0x3;
		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	  	  }

//	  if(0<HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
//	  		  TxHeader.StdId = 0x204;
//	  		  TxHeader.RTR = CAN_RTR_DATA;
//	  		  TxHeader.IDE = CAN_ID_STD;
//	  		  TxHeader.DLC = 8;
//	  		  TxHeader.TransmitGlobalTime = DISABLE;
//	  		  TxData[0] = 0x4;
//	  		  TxData[1] = 0x4;
//	  		  TxData[2] = 0x4;
//	  		  TxData[3] = 0x4;
//	  		  TxData[4] = 0x4;
//	  		  TxData[5] = 0x4;
//	  		  TxData[6] = 0x4;
//	  		  TxData[7] = 0x4;
//	  		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
//	  	  }
//
//	  if(0<HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
//	  		  TxHeader.StdId = 0x205;
//	  		  TxHeader.RTR = CAN_RTR_DATA;
//	  		  TxHeader.IDE = CAN_ID_STD;
//	  		  TxHeader.DLC = 8;
//	  		  TxHeader.TransmitGlobalTime = DISABLE;
//	  		  TxData[0] = 0x5;
//	  		  TxData[1] = 0x5;
//	  		  TxData[2] = 0x5;
//	  		  TxData[3] = 0x5;
//	  		  TxData[4] = 0x5;
//	  		  TxData[5] = 0x5;
//	  		  TxData[6] = 0x5;
//	  		  TxData[7] = 0x5;
//	  		  HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
//	  	  }
//	  printf("%d,%d,%d,%d,%d,%d,%d,%d",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
