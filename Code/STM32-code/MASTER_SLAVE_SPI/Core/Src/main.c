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

#include "SPI_functions.h"
#include <stdio.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void restart_spi(){
	HAL_SPI_DMAStop(&hspi1);
    __HAL_RCC_SPI1_FORCE_RESET();

    __HAL_RCC_SPI1_RELEASE_RESET();

    HAL_SPI_Init(&hspi1);
    HAL_Delay(1);

    HAL_SPI_Receive_DMA(&hspi1, spi_receive_confirmation_buffer, BUFFER_RECEIVE_CONFIRMATION_SIZE);
}



void handle_slave_state(uint8_t state) {
    switch(state) {
		case HOMMING_STATE:
			HAL_Delay(1000);
			//perform_homing();
			break;
		case MOVING_STATE:
			HAL_Delay(3000);
			//perform_movement();
			break;
		case STOP_STATE:
			//perform_stop();
			break;
		case SENDING_STATE:
			//send_status();
			break;
		case CHAGING_MOVEMENT_VALUES_STATE:
			//change_movement_values();
			break;
		case DIAGNOSTIC_STATE:
			//perform_diagnostics();
			break;
		default:
			// Handle unknown state if necessary
			break;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  char debug_buffer[248];
  HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive_DMA(&hspi1, spi_frame_buffer,BUFFER_FRAME_SIZE);
  sprintf(debug_buffer, "SPI START \r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  switch(SPI_state){
		  case SENDING_CONFIRMATION:
		  {
			  SPI_state = CONFIRMATION_RECEIVED;
			  // Przygotuj bufor do wysyłki (jeśli nie został wcześniej przygotowany)
			  // copying_to_buffer_confirmation_frame(&SPI_message);

			  // Ile bajtów wysyłamy: start + slave_id + command + length + dane + end_byte
			  uint8_t bytes_to_send = 4 + SPI_message.send_confirm.length + 1; // 4: start+slave+cmd+length, +1: end_byte

			  // === DEBUG: Wypisz informację o stanie i ilości bajtów ===
			  sprintf(debug_buffer, "[TX] Sending confirmation frame (%d bytes)\r\n", bytes_to_send);
			  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

			  // === DEBUG: Wypisz zawartość ramki ===
			  sprintf(debug_buffer, "[TX] Frame data: ");
			  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

			  for (uint8_t i = 0; i < bytes_to_send; i++) {
				  sprintf(debug_buffer, "%02X ", spi_send_confirmation_buffer[i]);
				  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
			  }
			  sprintf(debug_buffer, "\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

			  //HAL_SPI_DMAStop(&hspi1);
			  //HAL_SPI_Receive_DMA(&hspi1, spi_receive_confirmation_buffer, BUFFER_RECEIVE_CONFIRMATION_SIZE);
			  // === Transmisja ramki do mastera ===
			  HAL_SPI_Transmit(&hspi1, spi_send_confirmation_buffer, bytes_to_send, 200);

			  restart_spi();
			  // === DEBUG: Potwierdzenie zakończenia transmisji ===
			  sprintf(debug_buffer, "[TX] Confirmation frame sent, waiting for ACK...\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);


			  // === DEBUG: Potwierdzenie przejścia do kolejnego stanu ===
			  sprintf(debug_buffer, "[STATE] -> CONFIRMATION_RECEIVED\r\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

			  //SPI_state = CONFIRMATION_RECEIVED;
			  break;
		  }

	  	  case PROCES_CONFRMATION:
	  		    SPI_state = MESSAGE_RECEIVED;
				sprintf(debug_buffer, "[RX] Confirmation frame received: \r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				uint8_t error_code = copying_from_buffer_confirmation_frame(&SPI_message);
				// Wypisanie otrzymanego kodu błędu
				sprintf(debug_buffer, "[RX] Confirmation parse result: error_code = %d\r\n", error_code);
				HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);


//				for (uint8_t i = 0; i < BUFFER_RECEIVE_CONFIRMATION_SIZE; i++) {
//					sprintf(debug_buffer, "%02X ", spi_receive_confirmation_buffer[i]);
//					HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
//					HAL_Delay(1);//DEBBUGING
//				}


				for (uint8_t i = 0; i < BUFFER_RECEIVE_CONFIRMATION_SIZE; i++) {
				    // Zbuduj tekst dla bieżącego bajtu
				    sprintf(debug_buffer, "%02X ", spi_receive_confirmation_buffer[i]);
				    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 50);

				    // Jeśli to ostatni bajt, dodaj znak końca linii
				    if (i == BUFFER_RECEIVE_CONFIRMATION_SIZE - 1) {
				        sprintf(debug_buffer, "--END--\r\n");
				        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), 50);
				    }
				}

				sprintf(debug_buffer, "--END--\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				sprintf(debug_buffer, "[RX] SPI RESET\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				restart_spi();

				// Sprawdzenie, czy potwierdzenie zostało zaakceptowane
				sprintf(debug_buffer, "[RX] Acknowledge flag value: %d\r\n", SPI_message.acknowledge_confirmation);
				HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				SPI_state = MESSAGE_RECEIVED;

				if (SPI_message.acknowledge_confirmation) {
				        sprintf(debug_buffer, "[STATE] ACK received -> Executing process_received_command()\r\n");
				        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				        process_received_command(&SPI_message);

				        // Reset acknowledgment flag
				        SPI_message.acknowledge_confirmation = 0;

				        sprintf(debug_buffer, "[STATE] Command processed, ACK flag cleared\r\n");
				        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
				    } else {
				        sprintf(debug_buffer, "[STATE] NACK received -> Skipping command processing\r\n");
				        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

				        // Można dodać więcej logiki np. retransmisję
				    }
		  break;

	  }
	  switch(SLAVE_STATE) {
	      case IDLE_STATE:
	          break;
	      case HOMMING_STATE:
	      case MANUAL_CONTROL_STATE:
	      case MOVING_STATE:
	      case STOP_STATE:
	      case SENDING_STATE:
	      case ERROR_STATE:
	      case CHAGING_MOVEMENT_VALUES_STATE:
	      case DIAGNOSTIC_STATE:
	    	  sprintf(debug_buffer, "[STATE] WE SET PIN IN HIGH\r\n");
	    	  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
	          HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_SET);
	          // Wywołanie odpowiedniej funkcji:
	          handle_slave_state(SLAVE_STATE);
	          break;
	  }
	  SLAVE_STATE = IDLE_STATE;

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|PULL_MANUAL_Pin|ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENDSTOP_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ENDSTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin PULL_MANUAL_Pin ENABLE_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|PULL_MANUAL_Pin|ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SLAVE_END_TASK_Pin */
  GPIO_InitStruct.Pin = SLAVE_END_TASK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLAVE_END_TASK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
#ifdef USE_FULL_ASSERT
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
