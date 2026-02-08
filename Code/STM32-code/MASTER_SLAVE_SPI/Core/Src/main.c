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

//#include "SPI_functions.h"
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
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */
#define HANDSHAKE_SIZE 1
#define FRAME_INFORMATION_FOR_ECHO 5 //this is suppoused to be start_byte+command+slave_id+frame_id+end_byte
#define DATA_SIZE   16

#define START_BYTE 0x55
#define SLAVE_ID  0x01
#define END_BYTE 0xAA
#define SYN_BYTE 0x97
#define SYN_ACK_BYTE 0x98
#define ACK_BYTE 0x99

// --- Struktura nagłówka ---
typedef struct {
    uint8_t start_byte;
    uint8_t length;
    uint8_t slave_id;
    uint8_t frame_id;
    uint8_t command;
    uint8_t data_size;
    uint8_t end_byte;
} Header_t;

#define HEADER_SIZE (sizeof(Header_t))

//communication buffers
uint8_t rx_handshake;
Header_t rx_header;  // Teraz jako struktura!
uint8_t rx_data[DATA_SIZE];
uint8_t rx_commit;
uint8_t tx_handshake;
uint8_t tx_buf[DATA_SIZE + FRAME_INFORMATION_FOR_ECHO]; //this is suppoused to be start_byte+command+slave_id+frame_id+end_byte

//action buffer
uint8_t data_frame[HEADER_SIZE + DATA_SIZE];

// --- Stany ---
typedef enum {
    WAIT_SYN,
    SEND_SYN_ACK,
    WAIT_ACK,
    WAIT_HEADER,
    SEND_VALIDATION_CODE,  // ← NOWY STAN po odebraniu headera
    WAIT_DATA,
    SEND_ECHO,
    WAIT_COMMIT
} SPI_ProcessStage;

volatile SPI_ProcessStage process_stage = WAIT_SYN;

// Bufor na kod walidacji (1 bajt)
uint8_t tx_validation_code;


// --- Funkcje pomocnicze ---
void start_spi_dma_receive(uint8_t* buffer, uint16_t size){
    HAL_SPI_Receive_DMA(&hspi1, buffer, size);
}

void send_spi_dma(uint8_t* buffer, uint16_t size){
    HAL_SPI_Transmit_DMA(&hspi1, buffer, size);
}

void prepare_echo(){
    tx_buf[0] = rx_header.start_byte;      // START_BYTE
    tx_buf[1] = rx_header.command;         // command
    tx_buf[2] = rx_header.slave_id;        // slave_id
    tx_buf[3] = rx_header.frame_id;        // frame_id
    memcpy(&tx_buf[4], rx_data, rx_header.data_size); // data
    tx_buf[4 + rx_header.data_size] = rx_header.end_byte; // END_BYTE
}


void clear_communication_buffors(){
    //Reset all values to default
    rx_handshake = 0;
    memset(&rx_header, 0, sizeof(rx_header));  // Wyzeruj całą strukturę
    memset(rx_data, 0, sizeof(rx_data));

    tx_handshake = 0;
    memset(tx_buf, 0, sizeof(tx_buf));

    rx_commit = 0;
}

void copy_and_clear_communication_buffors(){
    //first do some copying data
    memcpy(data_frame, &rx_header, HEADER_SIZE);  // Kopiuj strukturę jako bajty
    memcpy(&data_frame[HEADER_SIZE], rx_data, DATA_SIZE);

    clear_communication_buffors();
}

typedef enum{
	OK = 1,
	WRONG_START_BYTE,
	WRONG_SLAVE_ID,
	WRONG_COMMAND,
	WRONG_DATA_SIZE,
	WRONG_END_BYTE
}Frame_Errors;

Frame_Errors frame_errors;

typedef enum{
	HOMMING = 100,
	MOVE_VIA_ANGLE,
	DIAGNOSTIC
}Commands;


//This function return value if its ends with no problme or there is somthing
Frame_Errors header_validation(){
	if(rx_header.start_byte != START_BYTE){
		return WRONG_START_BYTE;
	}
	if(rx_header.slave_id != SLAVE_ID){
		return WRONG_SLAVE_ID;
	}
	if(rx_header.command > DIAGNOSTIC || rx_header.command < HOMMING){
		return WRONG_COMMAND;
	}
	if(rx_header.data_size > DATA_SIZE){
		return WRONG_DATA_SIZE;
	}
	if(rx_header.end_byte != END_BYTE){
		return WRONG_END_BYTE;
	}

	return OK;
}

typedef enum{
    START,
    RUNNING,
    STOP
}Timer_State;

Timer_State timer_state = START;

void timer_reset(){
    HAL_TIM_Base_Stop_IT(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    timer_state = START;
}

// --- EXTI Callback (CS pin) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin != GPIO_PIN_4) return;

    if (timer_state == START) {
        HAL_TIM_Base_Start_IT(&htim1);
        timer_state = RUNNING;
    }

    HAL_SPI_DMAStop(&hspi1);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    switch(process_stage){
        case WAIT_SYN:
            start_spi_dma_receive(&rx_handshake, 1);
            break;

        case SEND_SYN_ACK:
            send_spi_dma(&tx_handshake, 1);
            break;

        case WAIT_ACK:
            start_spi_dma_receive(&rx_handshake, 1);
            break;

        case WAIT_HEADER:
            start_spi_dma_receive((uint8_t*)&rx_header, HEADER_SIZE);
            break;

        case SEND_VALIDATION_CODE:  // ← Wyślij kod walidacji headera
            send_spi_dma(&tx_validation_code, 1);
            break;

        case WAIT_DATA:
            start_spi_dma_receive(rx_data, rx_header.data_size);
            break;

        case SEND_ECHO:
            send_spi_dma(tx_buf, rx_header.data_size + FRAME_INFORMATION_FOR_ECHO);
            break;

        case WAIT_COMMIT:
            start_spi_dma_receive(&rx_commit, 1);
            break;
    }
}



// --- DMA RX Complete ---
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi != &hspi1) return;

    __HAL_SPI_CLEAR_OVRFLAG(hspi);

    switch(process_stage){
        case WAIT_SYN:
            if(rx_handshake == SYN_BYTE){
                tx_handshake = SYN_ACK_BYTE;
                process_stage = SEND_SYN_ACK;
            }
            break;

        case WAIT_ACK:
            process_stage = WAIT_HEADER;
            break;

        case WAIT_HEADER:
            // Waliduj header i zapisz kod do wysłania
            frame_errors = header_validation();
            tx_validation_code = (uint8_t)frame_errors;

            process_stage = SEND_VALIDATION_CODE;
            break;

        case WAIT_DATA:
            prepare_echo();
            process_stage = SEND_ECHO;
            break;

        case WAIT_COMMIT:
            process_stage = WAIT_SYN;
            timer_state = START;
            copy_and_clear_communication_buffors();
            break;
    }

    timer_reset();
}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi != &hspi1) return;

    if(process_stage == SEND_SYN_ACK){
        process_stage = WAIT_ACK;
    }
    else if(process_stage == SEND_VALIDATION_CODE){
        // Po wysłaniu kodu walidacji:
        if(frame_errors == OK){
            // OK - kontynuuj odbieranie danych
            process_stage = WAIT_DATA;
        } else {
            // BŁĄD - przerwij komunikację, wróć do początku
            clear_communication_buffors();
            process_stage = WAIT_SYN;
            timer_state = START;
        }
    }
    else if(process_stage == SEND_ECHO){
        process_stage = WAIT_COMMIT;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim1){
        //we need to clear before doing anything
        clear_communication_buffors();
        //change status of course
        process_stage = WAIT_SYN;
        timer_state = START;
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
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
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SLAVE_END_TASK_Pin */
  GPIO_InitStruct.Pin = SLAVE_END_TASK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLAVE_END_TASK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
