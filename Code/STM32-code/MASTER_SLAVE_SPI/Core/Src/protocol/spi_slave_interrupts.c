/**
 * @file    spi_slave_interrupts.c
 * @brief   SPI interrupt handlers
 * @details Contains all interrupt callbacks for SPI communication
 */

#include "protocol/spi_slave_protocol.h"
#include "spi_slave_registers.h"

/*******************************************************************************
 * GPIO EXTI INTERRUPT CALLBACK (Chip Select)
 ******************************************************************************/

/**
 * @brief GPIO EXTI interrupt callback (Chip Select pin)
 * @param GPIO_Pin GPIO pin that triggered interrupt
 * @details Handles CS pin falling edge - starts appropriate DMA transfer
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_4) return;

    /* Start timeout timer on first CS edge */
    if (timer_state == TIMER_START) {
    	HAL_TIM_Base_Stop_IT(&htim1);
    	__HAL_TIM_SET_COUNTER(&htim1, 0);
        HAL_TIM_Base_Start_IT(&htim1);
        timer_state = TIMER_RUNNING;
    }

    /* Stop any ongoing DMA transfer and clear flags */
    HAL_SPI_DMAStop(&hspi1);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);

    /* Start appropriate DMA transfer based on current stage */
    switch (process_stage) {
        case WAIT_SYN:
            spi_start_dma_receive(&rx_handshake, HANDSHAKE_SIZE);
            break;

        case SEND_SYN_ACK:
            spi_start_dma_transmit(&tx_handshake, HANDSHAKE_SIZE);
            break;

        case WAIT_ACK:
            spi_start_dma_receive(&rx_handshake, HANDSHAKE_SIZE);
            break;

        case WAIT_HEADER:
            spi_start_dma_receive((uint8_t*)&rx_header, HEADER_SIZE);
            break;

        case SEND_VALIDATION_CODE:
            spi_start_dma_transmit(&tx_validation_code, 1);
            break;

        /* Write path: Master writes to slave */
        case WAIT_DATA: {
            uint16_t data_size = ((uint16_t)rx_header.data_size[0] << 8) | 
                                  rx_header.data_size[1];
            spi_start_dma_receive(rx_data, data_size + SIZE_OF_CRC);
            break;
        }

        case SEND_COMMIT:
            spi_start_dma_transmit(tx_commit, COMMIT_FRAME_SIZE);
            break;

        /* Read path: Master reads from slave */
        case SEND_DATA: {
            uint16_t reg_addr = ((uint16_t)rx_header.register_address[0] << 8) | 
                                 rx_header.register_address[1];
            uint16_t data_size = ((uint16_t)rx_header.data_size[0] << 8) | 
                                  rx_header.data_size[1];

            /* Copy register data to transmit buffer */
            for (uint8_t i = 0; i < data_size; i++) {
                void *reg_ptr = register_map[reg_addr].read_function(NULL, i);
                tx_buf[i] = *((uint8_t*)reg_ptr);
            }

            /* Append CRC to data */
            append_crc_to_buffer(tx_buf, data_size);

            /* Send data + CRC */
            spi_start_dma_transmit(tx_buf, data_size + SIZE_OF_CRC);
            break;
        }

        case WAIT_COMMIT:
            spi_start_dma_receive(rx_commit, COMMIT_FRAME_SIZE);
            break;
    }
}

/*******************************************************************************
 * SPI RX COMPLETE CALLBACK
 ******************************************************************************/

/**
 * @brief SPI DMA receive complete callback
 * @param hspi SPI handle pointer
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) return;

    __HAL_SPI_CLEAR_OVRFLAG(hspi);

    switch (process_stage) {
        case WAIT_SYN:
            /* Check if received SYN byte */
            if (rx_handshake == SYN_BYTE) {
                tx_handshake = SYN_ACK_BYTE;
                process_stage = SEND_SYN_ACK;
            }
            break;

        case WAIT_ACK:
            /* ACK received, proceed to header */
            if (rx_handshake == ACK_BYTE) {
                process_stage = WAIT_HEADER;
            }
            break;

        case WAIT_HEADER:
            /* Validate header and prepare validation code */
            last_error = validate_header();
            tx_validation_code = (uint8_t)last_error;
            process_stage = SEND_VALIDATION_CODE;
            break;

        case WAIT_DATA: {
            /* Data received, validate CRC */
            uint16_t reg_addr = ((uint16_t)rx_header.register_address[0] << 8) | 
                                 rx_header.register_address[1];
            uint16_t data_size = ((uint16_t)rx_header.data_size[0] << 8) | 
                                  rx_header.data_size[1];

            Frame_Errors_t data_error = validate_data(rx_data, data_size + SIZE_OF_CRC);

            if (data_error == OK_FRAME) {
                /* Write data to register */
                for (uint8_t i = 0; i < data_size; i++) {
                    register_map[reg_addr].write_function(NULL, i, rx_data[i]);
                }
                prepare_commit_frame(OK_FRAME);
            } else {
                /* CRC error */
                prepare_commit_frame(data_error);
            }

            process_stage = SEND_COMMIT;
            break;
        }

        case WAIT_COMMIT:
            /* Validate commit frame */
            if (rx_commit[0] == START_BYTE &&
                rx_commit[1] == COMMIT_BYTE &&
                rx_commit[3] == END_BYTE) {
                
                /* Check error code from master */
                Frame_Errors_t master_error = (Frame_Errors_t)rx_commit[2];
                
                if (master_error == OK_FRAME) {
                    /* Transaction successful */
                } else {
                    /* Master reported error */
                }
            }

            /* Reset communication */
            clear_communication_buffers();
            process_stage = WAIT_SYN;
            timer_state = TIMER_START;
            break;
    }

    timer_reset();
}

/*******************************************************************************
 * SPI TX COMPLETE CALLBACK
 ******************************************************************************/

/**
 * @brief SPI DMA transmit complete callback
 * @param hspi SPI handle pointer
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) return;

    switch (process_stage) {
        case SEND_SYN_ACK:
            /* SYN_ACK sent, wait for ACK */
            process_stage = WAIT_ACK;
            break;

        case SEND_VALIDATION_CODE:
            /* Validation code sent */
            if (last_error == OK_FRAME) {
                /* Header valid - proceed based on operation type */
                if (rx_header.operation_type == OPERATION_WRITE) {
                    process_stage = WAIT_DATA;
                } else if (rx_header.operation_type == OPERATION_READ) {
                    process_stage = SEND_DATA;
                }
            } else {
                /* Header invalid - abort and reset */
                clear_communication_buffers();
                process_stage = WAIT_SYN;
                timer_state = TIMER_START;
            }
            break;

        case SEND_DATA:
            /* Data sent to master, wait for commit */
            process_stage = WAIT_COMMIT;
            break;

        case SEND_COMMIT:
            /* Commit sent - transaction complete */
            clear_communication_buffers();
            process_stage = WAIT_SYN;
            timer_state = TIMER_START;
            break;
    }

    timer_reset();
}

/*******************************************************************************
 * TIMER PERIOD ELAPSED CALLBACK (Timeout)
 ******************************************************************************/

/**
 * @brief Timer period elapsed callback (timeout handler)
 * @param htim Timer handle pointer
 * @details Resets communication on timeout
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1) {
        /* Timeout occurred - reset communication */
        clear_communication_buffers();
        process_stage = WAIT_SYN;
        timer_state = TIMER_START;
    }
}
