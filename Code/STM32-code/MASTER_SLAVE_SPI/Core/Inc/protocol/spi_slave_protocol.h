/**
 * @file    spi_slave_protocol.h
 * @brief   SPI protocol functions header
 * @details Function prototypes for SPI communication protocol
 */

#ifndef SPI_SLAVE_PROTOCOL_H
#define SPI_SLAVE_PROTOCOL_H

#include <protocol/spi_slave_types.h>
#include "main.h"

/*******************************************************************************
 * EXTERNAL PERIPHERAL HANDLES
 ******************************************************************************/

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern CRC_HandleTypeDef hcrc;

/*******************************************************************************
 * COMMUNICATION STATE VARIABLES
 ******************************************************************************/

extern volatile SPI_ProcessStage_t process_stage;
extern volatile uint16_t old_frame_id;
extern volatile Timer_State_t timer_state;
extern volatile Frame_Errors_t last_error;

/*******************************************************************************
 * COMMUNICATION BUFFERS
 ******************************************************************************/

/* RX Buffers */
extern uint8_t rx_handshake;
extern Header_t rx_header;
extern uint8_t rx_data[DATA_SIZE];
extern uint8_t rx_commit[COMMIT_FRAME_SIZE];

/* TX Buffers */
extern uint8_t tx_handshake;
extern uint8_t tx_buf[DATA_SIZE + SIZE_OF_CRC];
extern uint8_t tx_commit[COMMIT_FRAME_SIZE];
extern uint8_t tx_validation_code;

/*******************************************************************************
 * DMA CONTROL FUNCTIONS
 ******************************************************************************/

void spi_start_dma_receive(uint8_t* buffer, uint16_t size);
void spi_start_dma_transmit(uint8_t* buffer, uint16_t size);

/*******************************************************************************
 * CRC FUNCTIONS
 ******************************************************************************/

uint32_t calculate_crc(uint8_t* data, uint16_t length);
void append_crc_to_buffer(uint8_t* buffer, uint16_t data_length);

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

Frame_Errors_t validate_header(void);
Frame_Errors_t validate_data(uint8_t* buffer, uint16_t length);

/*******************************************************************************
 * FRAME PREPARATION FUNCTIONS
 ******************************************************************************/

void prepare_commit_frame(Frame_Errors_t error_code);

/*******************************************************************************
 * BUFFER MANAGEMENT FUNCTIONS
 ******************************************************************************/

void clear_communication_buffers(void);

/*******************************************************************************
 * TIMER CONTROL FUNCTIONS
 ******************************************************************************/

void timer_reset(void);

/*******************************************************************************
 * INTERRUPT CALLBACK PROTOTYPES
 ******************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* SPI_SLAVE_PROTOCOL_H */
