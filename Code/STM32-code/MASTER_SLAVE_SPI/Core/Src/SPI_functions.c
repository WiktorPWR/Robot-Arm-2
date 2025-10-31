/*
 * SPI_functions.c
 *
 *  Created on: Oct 11, 2025
 *      Author: ostro
 */

#include "main.h"
#include "SPI_functions.h"
#include "data_templates.h"

//0xB00B5EE5

extern UART_HandleTypeDef huart2;
char debug_buffer[100];


uint8_t data_buffer[MAX_DATA_SIZE];
volatile uint8_t clear_spi_frame;

uint8_t spi_frame_buffer[BUFFER_FRAME_SIZE];
uint8_t spi_receive_confirmation_buffer[BUFFER_RECEIVE_CONFIRMATION_SIZE];
uint8_t spi_send_confirmation_buffer[BUFFER_FRAME_SIZE];

volatile struct message SPI_message = {0};
volatile enum communication_states SPI_state = MESSAGE_RECEIVED;
volatile enum slave_state SLAVE_STATE = IDLE_STATE;

#define START_BYTE_POSITION 0
#define SLAVE_ID_POSITION 1

#define COMMAND_POSITION 2
#define LENGTH_POSITION 3
#define DATA_POSITION 4
#define END_BYTE_POSITION_FRAME 24

#define DATA_POSITION_CONFIRMATION 1
#define END_BYTE_POSITION_CONFIRMATION 3


/**
 * @brief Copies and validates a normal data frame from the SPI receive buffer.
 *
 * This function reads bytes directly from the SPI RX buffer and populates the
 * provided @ref message structure. It validates the frame structure (start byte,
 * command, data length, end byte) and copies payload data to the frame buffer.
 *
 * @param[in]  hspi1       Pointer to the active SPI handle structure.
 * @param[out] SPI_message Pointer to the message structure to be filled with received data.
 *
 * @return Status code indicating result:
 *         - @ref NO_ERROR                on success
 *         - @ref WRONG_START_BYTE        if the start byte is invalid
 *         - @ref WRONG_LENGTH_VALUE      if message length does not match expected
 *         - @ref NO_SUCH_COMMAND_VALUE   if the command code is unknown
 *         - @ref WRONG_END_BYTE          if the end byte is invalid
 */
uint8_t copying_from_buffer_normal_frame(struct message* SPI_message){
	volatile uint8_t *rx_buf = spi_frame_buffer;

	// --- Start byte ---
	SPI_message->frame.start_byte = rx_buf[START_BYTE_POSITION];
	if(SPI_message->frame.start_byte != START_BYTE){
		return WRONG_START_BYTE;
	}

	// --- Command i length ---
	SPI_message->frame.command = rx_buf[COMMAND_POSITION];
	SPI_message->frame.length  = rx_buf[LENGTH_POSITION];

	// --- Sprawdzenie długości dla znanych komend ---
	switch(SPI_message->frame.command){
		case START_HOMMING_COMMAND:
			if(SPI_message->frame.length != HOMMING_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case START_MOVING_COMMAND:
			if(SPI_message->frame.length != MOVING_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case GET_STATUS_COMMAND:
			if(SPI_message->frame.length != STATUS_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case STOP_COMMAND:
			if(SPI_message->frame.length != STOP_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case CHANGE_MOVEMENT_VALUES:
			if(SPI_message->frame.length != MOVEMENT_VALUES_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case START_DIAGNOSTIC_COMMAND:
			if(SPI_message->frame.length != DIAGNOSTICS_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		case ACCEPT_CONFIRMATION_COMMAND:
			if(SPI_message->frame.length != ACCEPT_CONFIRMATION_SIZE_OF_DATA) return WRONG_LENGTH_VALUE;
			break;
		default:
			return NO_SUCH_COMMAND_VALUE;
	}

	// --- Kopiowanie danych tylko jeśli length > 0 ---
	if(SPI_message->frame.length > 0){
		memcpy(SPI_message->frame.data, &rx_buf[DATA_POSITION], SPI_message->frame.length);
	}

	// --- End byte ---
	SPI_message->frame.end_byte = rx_buf[DATA_POSITION + SPI_message->frame.length];
	if(SPI_message->frame.end_byte != END_BYTE){
		return WRONG_END_BYTE;
	}

	return NO_ERROR;
}



/**
 * @brief Prepares a response confirmation frame for the SPI master.
 *
 * This function populates the confirmation frame within the provided
 * @ref message structure based on the specified error code. If no error
 * is indicated, it copies the received data back into the confirmation frame.
 *
 * @param[in,out] SPI_message Pointer to the message structure to fill.
 * @param[in]     ERROR_CODE  Error code indicating success or type of failure.
 */
void preparing_response_for_master(struct message* SPI_message, uint8_t ERROR_CODE){
    SPI_message->send_confirm.start_byte = START_BYTE;

    if(ERROR_CODE == NO_ERROR){
        // Jeśli trzeba zresetować slave przed nowym poleceniem
        if(SPI_message->frame.command != ACCEPT_CONFIRMATION_COMMAND && (GPIOB->ODR & GPIO_PIN_5)){
            SPI_message->send_confirm.command = BEFOR_USING_ANY_NEW_COMMAND_RESET_SLAVE;
        } else {
            SPI_message->send_confirm.command = NO_ERROR;
        }

        // Kopiujemy dane tylko jeśli length > 0
        if(SPI_message->frame.length > 0){
            SPI_message->send_confirm.length = SPI_message->frame.length;
            memcpy(SPI_message->send_confirm.data, SPI_message->frame.data, SPI_message->send_confirm.length);
        } else {
            SPI_message->send_confirm.length = 0;  // brak danych
        }

    } else {
        // Wystąpił błąd → brak danych do wysłania
        SPI_message->send_confirm.command = ERROR_CODE;
        SPI_message->send_confirm.length  = 0;
        memset(SPI_message->send_confirm.data, 0, MAX_DATA_SIZE);
    }

    SPI_message->send_confirm.slave_id = SLAVE_ID;
    SPI_message->send_confirm.end_byte = END_BYTE;
}


/**
 * @brief Copies and validates a confirmation frame from the SPI receive buffer.
 *
 * This function parses a short confirmation frame (start, data, end bytes)
 * and updates the @ref message structure with acknowledgment information.
 *
 * @param[in]  hspi1       Pointer to the SPI handle structure.
 * @param[out] SPI_message Pointer to the message structure that stores confirmation info.
 *
 * @return Status code indicating result:
 *         - @ref NO_ERROR                on success
 *         - @ref WRONG_CONFIRMATION_FRAME if frame structure is invalid
 */
uint8_t copying_from_buffer_confirmation_frame(struct message* SPI_message){
	uint8_t *rx_buf = spi_receive_confirmation_buffer;
	SPI_message->receive_confirm.start_byte = rx_buf[START_BYTE_POSITION];
	SPI_message->receive_confirm.slave_id   = rx_buf[SLAVE_ID_POSITION];
	SPI_message->receive_confirm.data       = rx_buf[DATA_POSITION_CONFIRMATION];
	SPI_message->receive_confirm.end_byte   = rx_buf[END_BYTE_POSITION_CONFIRMATION];


	if(SPI_message->receive_confirm.start_byte != START_BYTE){
		return WRONG_START_BYTE_CONFIRMATION;
	}

	if(SPI_message->receive_confirm.end_byte != END_BYTE){
		return WRONG_END_BYTE_CONFIRMATION;
	}

	if(SPI_message->receive_confirm.data == ACKNOWLEDGMENT_OK){
		SPI_message->acknowledge_confirmation = 1;
	} else {
		SPI_message->acknowledge_confirmation = 0;
	}

	return NO_ERROR;
}



/**
 * @brief Copies a confirmation frame into the SPI transmit buffer.
 *
 * This function prepares the transmit buffer with the confirmation frame
 * data from the provided @ref message structure for sending back to the master.
 *
 * @param[in]  hspi1       Pointer to the SPI handle structure.
 * @param[out] SPI_message Pointer to the message structure containing confirmation data.
 *
 * @return Status code indicating result:
 *         - @ref NO_ERROR on success
 */
uint8_t copying_to_buffer_confirmation_frame(struct message* SPI_message){
    uint8_t *tx_buf = spi_send_confirmation_buffer;

    tx_buf[START_BYTE_POSITION] = SPI_message->send_confirm.start_byte;
    tx_buf[SLAVE_ID_POSITION] = SPI_message->send_confirm.slave_id;
    tx_buf[COMMAND_POSITION] = SPI_message->send_confirm.command;
    tx_buf[LENGTH_POSITION] = SPI_message->send_confirm.length;

    if(SPI_message->send_confirm.length > 0){
        // Kopiujemy tylko tyle bajtów, ile wynosi length
        memcpy(&tx_buf[DATA_POSITION], (void*)SPI_message->send_confirm.data, SPI_message->send_confirm.length);
        tx_buf[DATA_POSITION + SPI_message->send_confirm.length] = SPI_message->send_confirm.end_byte;
    } else {
        // Brak danych, od razu end byte
        tx_buf[DATA_POSITION] = SPI_message->send_confirm.end_byte;
    }

    return NO_ERROR;
}


/**
 * @brief Clears all data within a SPI message structure.
 *
 * This function resets all fields within a @ref message structure to zero,
 * including command metadata and data buffer content.
 *
 * @param[in,out] SPI_message Pointer to the message structure to clear.
 */
void clear_spi_message(struct message* SPI_message){
	SPI_message->frame.start_byte = 0;
	SPI_message->frame.command    = 0;
	SPI_message->frame.length     = 0;
	SPI_message->frame.end_byte   = 0;
	//Free allocated memory for data field
	memset(SPI_message->frame.data,0,MAX_DATA_SIZE);

	//Clear receiving confirmation frame
	SPI_message->receive_confirm.start_byte = 0;
 	SPI_message->receive_confirm.data       = 0;
	SPI_message->receive_confirm.end_byte   = 0;

	//Clear sending confirmation frame
	SPI_message->send_confirm.start_byte = 0;
	memset(SPI_message->send_confirm.data,0,MAX_DATA_SIZE);
	SPI_message->send_confirm.end_byte   = 0;

	memset(spi_frame_buffer,0,BUFFER_FRAME_SIZE);
	memset(spi_receive_confirmation_buffer,0,BUFFER_RECEIVE_CONFIRMATION_SIZE);
	memset(spi_send_confirmation_buffer,0,BUFFER_FRAME_SIZE);

}


/**
 * @brief Processes a fully received SPI command message.
 *
 * Based on the received command code, this function calls appropriate handlers
 * to perform actions such as homing, motion control, or diagnostic routines.
 *
 * @param[in,out] SPI_message Pointer to the received message to process.
 */
void process_received_command(struct message* SPI_message){

	switch(SPI_message->frame.command){
		case START_HOMMING_COMMAND:
			SLAVE_STATE = HOMMING_STATE;
			//Call homming function
			break;
		case START_MOVING_COMMAND:
			SLAVE_STATE = MOVING_STATE;
			//Call moving function with parameters from data field
			break;
		case STOP_COMMAND:
			SLAVE_STATE = STOP_STATE;
			//Call stop function
			break;
		case GET_STATUS_COMMAND:
			SLAVE_STATE = SENDING_STATE;
			//Call get status function
			break;
		case CHANGE_MOVEMENT_VALUES:
			SLAVE_STATE = CHAGING_MOVEMENT_VALUES_STATE;
			//Call change movement values function with parameters from data field
			break;
		case START_DIAGNOSTIC_COMMAND:
			SLAVE_STATE = DIAGNOSTIC_STATE;
			//Call start diagnostic function
			break;
		case ACCEPT_CONFIRMATION_COMMAND:
			SLAVE_STATE = IDLE_STATE;
			HAL_GPIO_WritePin(SLAVE_END_TASK_GPIO_Port, SLAVE_END_TASK_Pin, GPIO_PIN_RESET);
		    break;
		default:
			SLAVE_STATE = IDLE_STATE;
			//Handle unknown command error
			break;
	}


}

uint8_t check_slave_address(){
	uint8_t address = spi_frame_buffer[1];
	if(address == SLAVE_ID){
		return 1;
	}else{
		return 0;
	}
}

//interupt called when SPI transmission is complete
// value sending confirmation will be handled in main loop
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    sprintf(debug_buffer, "\r\n[INT] SPI RX interrupt triggered\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

    if (check_slave_address() != 1) {
        sprintf(debug_buffer, "[INT] Ignored frame (wrong slave ID)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
        return;
    }

    switch (SPI_state)
    {
        // ===================== CASE: MESSAGE_RECEIVED =====================
        case MESSAGE_RECEIVED:
        {
            uint8_t error_code = copying_from_buffer_normal_frame(&SPI_message);
            preparing_response_for_master(&SPI_message, error_code);
            copying_to_buffer_confirmation_frame(&SPI_message);

            // --- Log odebranej ramki ---
            sprintf(debug_buffer, "[RX] Frame received (%d bytes): ", BUFFER_FRAME_SIZE);
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            for (uint8_t i = 0; i < BUFFER_FRAME_SIZE; i++) {
                sprintf(debug_buffer, "%02X ", spi_frame_buffer[i]);
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            }
            sprintf(debug_buffer, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

            if (error_code == NO_ERROR)
            {
                sprintf(debug_buffer, "[RX] Frame OK: CMD=0x%02X LEN=%d\r\n",
                        SPI_message.frame.command, SPI_message.frame.length);
            }
            else
            {
                sprintf(debug_buffer, "[RX] Frame ERROR: CODE=%d CMD=0x%02X\r\n",
                        error_code, SPI_message.frame.command);
            }
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

            // --- Log przygotowanej ramki do wysłania ---
            sprintf(debug_buffer, "[TX] Response to send: ");
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            for (uint8_t i = 0; i < BUFFER_FRAME_SIZE; i++) {
                sprintf(debug_buffer, "%02X ", spi_send_confirmation_buffer[i]);
                HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            }
            sprintf(debug_buffer, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);

            SPI_state = SENDING_CONFIRMATION;
            break;
        }

        case CONFIRMATION_RECEIVED:
		{
			SPI_state = PROCES_CONFRMATION;
			break;
		}
        // ===================== DEFAULT =====================
        default:
        {
            sprintf(debug_buffer, "[ERR] Unknown SPI_state = %d\r\n", SPI_state);
            HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            break;
        }

    }
}

