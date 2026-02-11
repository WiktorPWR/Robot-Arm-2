/**
 * @file    spi_slave_protocol.c
 * @brief   SPI protocol implementation - Basic functions
 * @details DMA control, CRC, validation, and buffer management
 */

#include "protocol/spi_slave_protocol.h"
#include "spi_slave_registers.h"
#include <string.h>

/*******************************************************************************
 * COMMUNICATION STATE VARIABLES (DEFINITIONS)
 ******************************************************************************/

volatile SPI_ProcessStage_t process_stage = WAIT_SYN;
volatile uint16_t old_frame_id = 0xFFFF;
volatile Timer_State_t timer_state = TIMER_START;
volatile Frame_Errors_t last_error = OK_FRAME;

/*******************************************************************************
 * COMMUNICATION BUFFERS (DEFINITIONS)
 ******************************************************************************/

/* RX Buffers */
uint8_t rx_handshake;
Header_t rx_header;
uint8_t rx_data[DATA_SIZE];
uint8_t rx_commit[COMMIT_FRAME_SIZE];

/* TX Buffers */
uint8_t tx_handshake;
uint8_t tx_buf[DATA_SIZE + SIZE_OF_CRC];
uint8_t tx_commit[COMMIT_FRAME_SIZE];
uint8_t tx_validation_code;

/*******************************************************************************
 * DMA CONTROL FUNCTIONS
 ******************************************************************************/

/**
 * @brief Start SPI DMA reception
 * @param buffer Pointer to receive buffer
 * @param size Number of bytes to receive
 */
void spi_start_dma_receive(uint8_t* buffer, uint16_t size)
{
    HAL_SPI_Receive_DMA(&hspi1, buffer, size);
}

/**
 * @brief Start SPI DMA transmission
 * @param buffer Pointer to transmit buffer
 * @param size Number of bytes to transmit
 */
void spi_start_dma_transmit(uint8_t* buffer, uint16_t size)
{
    HAL_SPI_Transmit_DMA(&hspi1, buffer, size);
}

/*******************************************************************************
 * CRC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Calculate CRC-32 for given data
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return Calculated CRC-32 value
 */
uint32_t calculate_crc(uint8_t* data, uint16_t length)
{
    uint32_t words = (length + 3) / 4;  /* Round up to word boundary */
    uint32_t crc_buffer[words];
    
    memset(crc_buffer, 0, sizeof(crc_buffer));
    memcpy(crc_buffer, data, length);
    
    __HAL_CRC_DR_RESET(&hcrc);
    return HAL_CRC_Calculate(&hcrc, crc_buffer, words);
}

/**
 * @brief Append CRC-32 to buffer
 * @param buffer Pointer to buffer (must have space for 4 extra bytes)
 * @param data_length Length of data (CRC will be appended after this)
 */
void append_crc_to_buffer(uint8_t* buffer, uint16_t data_length)
{
    uint32_t crc = calculate_crc(buffer, data_length);
    
    buffer[data_length + 0] = (crc >> 24) & 0xFF;
    buffer[data_length + 1] = (crc >> 16) & 0xFF;
    buffer[data_length + 2] = (crc >> 8) & 0xFF;
    buffer[data_length + 3] = crc & 0xFF;
}

/*******************************************************************************
 * VALIDATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Validate received header structure
 * @return Frame_Errors_t validation result code
 */
Frame_Errors_t validate_header(void)
{
    /* Check start byte */
    if (rx_header.start_byte != START_BYTE) {
        return WRONG_START_BYTE;
    }

    /* Check end byte */
    if (rx_header.end_byte != END_BYTE) {
        return WRONG_END_BYTE;
    }

    /* Check slave ID */
    if (rx_header.slave_id != SLAVE_ID) {
        return WRONG_SLAVE_ID;
    }

    /* Extract and validate frame ID */
    uint16_t frame_id = ((uint16_t)rx_header.frame_id[0] << 8) | rx_header.frame_id[1];
    if (frame_id == old_frame_id) {
        return WRONG_FRAME_ID;
    }

    /* Extract register address */
    uint16_t reg_addr = ((uint16_t)rx_header.register_address[0] << 8) | 
                         rx_header.register_address[1];
    if (reg_addr >= REG_COUNT) {
        return WRONG_REGISTER;
    }

    /* Extract data size */
    uint16_t data_size = ((uint16_t)rx_header.data_size[0] << 8) |
                          rx_header.data_size[1];

    /* Check if data size exceeds buffer or register size */
    if (data_size > DATA_SIZE || data_size > register_map[reg_addr].size) {
        return WRONG_DATA_SIZE;
    }


    if (data_size == 0){
    	return WRONG_DATA_SIZE;
    }


    /* Validate operation type against register permissions */
    if (rx_header.operation_type == OPERATION_READ) {
        if (!(register_map[reg_addr].flags & REGISTER_READ)) {
            return WRONG_OPERATION;
        }
    } else if (rx_header.operation_type == OPERATION_WRITE) {
        if (!(register_map[reg_addr].flags & REGISTER_WRITE)) {
            return WRONG_OPERATION;
        }
    } else {
        return WRONG_OPERATION;
    }

    __HAL_CRC_DR_RESET(&hcrc);

    /* Validate CRC */
    uint32_t crc_rx = ((uint32_t)rx_header.crc[0] << 24) |
                      ((uint32_t)rx_header.crc[1] << 16) |
                      ((uint32_t)rx_header.crc[2] << 8) |
                      rx_header.crc[3];

    /* Calculate CRC for header (excluding start, end, and CRC itself) */
    uint32_t crc_calc = calculate_crc(&rx_header.slave_id, HEADER_SIZE_WITHOUT_CRC);

    if (crc_rx != crc_calc) {
        return WRONG_CRC_VALUE;
    }

    /* All checks passed - update frame ID */
    old_frame_id = frame_id;

    return OK_FRAME;
}

/**
 * @brief Validate received data with CRC check
 * @param buffer Pointer to data buffer (includes data + 4 byte CRC)
 * @param length Total length (data + CRC)
 * @return Frame_Errors_t validation result
 */
Frame_Errors_t validate_data(uint8_t* buffer, uint16_t length)
{
    if (length < SIZE_OF_CRC) {
        return WRONG_CRC_VALUE;
    }

    /* Extract received CRC (last 4 bytes) */
    uint32_t crc_rx = ((uint32_t)buffer[length - 4] << 24) |
                      ((uint32_t)buffer[length - 3] << 16) |
                      ((uint32_t)buffer[length - 2] << 8) |
                      ((uint32_t)buffer[length - 1]);

    /* Calculate CRC for data (excluding CRC bytes) */
    uint16_t data_len = length - SIZE_OF_CRC;
    uint32_t crc_calc = calculate_crc(buffer, data_len);

    return (crc_rx == crc_calc) ? OK_FRAME : WRONG_CRC_VALUE;
}

/*******************************************************************************
 * FRAME PREPARATION FUNCTIONS
 ******************************************************************************/

/**
 * @brief Prepare commit confirmation frame with error code
 * @param error_code Error code to include in commit frame
 */
void prepare_commit_frame(Frame_Errors_t error_code)
{
    tx_commit[0] = START_BYTE;
    tx_commit[1] = COMMIT_BYTE;
    tx_commit[2] = (uint8_t)error_code;
    tx_commit[3] = END_BYTE;
}

/*******************************************************************************
 * BUFFER MANAGEMENT FUNCTIONS
 ******************************************************************************/

/**
 * @brief Clear all communication buffers
 */
void clear_communication_buffers(void)
{
    /* Clear RX buffers */
    rx_handshake = 0;
    memset(&rx_header, 0, sizeof(rx_header));
    memset(rx_data, 0, sizeof(rx_data));
    memset(rx_commit, 0, sizeof(rx_commit));

    /* Clear TX buffers */
    tx_handshake = 0;
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(tx_commit, 0, sizeof(tx_commit));
    tx_validation_code = 0;
}

/*******************************************************************************
 * TIMER CONTROL FUNCTIONS
 ******************************************************************************/

/**
 * @brief Reset and stop timeout timer
 */
void timer_reset(void)
{
    HAL_TIM_Base_Stop_IT(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    timer_state = TIMER_START;
}
