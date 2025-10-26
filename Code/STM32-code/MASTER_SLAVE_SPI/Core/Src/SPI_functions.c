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


uint8_t data_buffer[MAX_DATA_SIZE];
volatile uint8_t clear_spi_frame;

uint8_t spi_frame_buffer[BUFFER_FRAME_SIZE];
uint8_t spi_receive_confirmation_buffer[BUFFER_RECEIVE_CONFIRMATION_SIZE];
uint8_t spi_send_confirmation_buffer[BUFFER_SEND_CONFIRMATION_SIZE];

volatile struct message SPI_message = {0};
volatile enum communication_states SPI_state = MESSAGE_RECEIVED;
volatile enum slave_state SLAVE_STATE = IDLE_STATE;

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

	//Copy start byte, command, length from rx buffer to message frame
	SPI_message->frame.start_byte = rx_buf[0];
	if(SPI_message->frame.start_byte != START_BYTE){
			return WRONG_START_BYTE;
	}


	//Copy command and length
	SPI_message->frame.command = rx_buf[1];
	SPI_message->frame.length = rx_buf[2];
	//Check lenght of message, length is size in bytes
	switch(SPI_message->frame.command){
		case START_HOMMING_COMMAND:
			if(SPI_message->frame.length != HOMMING_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case START_MOVING_COMMAND:
			if(SPI_message->frame.length != MOVING_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case GET_STATUS_COMMAND:
			if(SPI_message->frame.length != STATUS_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case STOP_COMMAND:
			if(SPI_message->frame.length != STOP_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case CHANGE_MOVEMENT_VALUES:
			if(SPI_message->frame.length != MOVEMENT_VALUES_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case START_DIAGNOSTIC_COMMAND:
			if(SPI_message->frame.length != DIAGNOSTICS_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		case ACCEPT_CONFIRMATION_COMMAND:
			if(SPI_message->frame.length != ACCEPT_CONFIRMATION_SIZE_OF_DATA){
				return WRONG_LENGTH_VALUE;
			}
			break;
		default:
			return NO_SUCH_COMMAND_VALUE;
	}

	//Copy data from rx buffer to message frame
	memcpy((void*)SPI_message->frame.data, &spi_frame_buffer[3], SPI_message->frame.length);
	SPI_message->frame.end_byte = spi_frame_buffer[3 + SPI_message->frame.length];

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
void preparing_response_for_master(struct message* SPI_message,uint8_t ERROR_CODE){
	SPI_message->send_confirm.start_byte = START_BYTE;

	if(ERROR_CODE == NO_ERROR){
		if(SPI_message->frame.command != ACCEPT_CONFIRMATION_COMMAND && (GPIOB->ODR & GPIO_PIN_5)){
				//Before any new command we need to reset slave
				SPI_message->send_confirm.command = BEFOR_USING_ANY_NEW_COMMAND_RESET_SLAVE;
		}else{
			SPI_message->send_confirm.command = NO_ERROR;
		}
		SPI_message->send_confirm.length  = SPI_message->frame.length;
		memcpy(SPI_message->send_confirm.data, SPI_message->frame.data, SPI_message->send_confirm.length);
	}else{
		//we have a problem
		SPI_message->send_confirm.command = ERROR_CODE;
		SPI_message->send_confirm.length  = 0;
		memset(SPI_message->send_confirm.data,0,MAX_DATA_SIZE);
	}


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
	SPI_message->receive_confirm.start_byte = rx_buf[0];
	SPI_message->receive_confirm.data       = rx_buf[1];
	SPI_message->receive_confirm.end_byte   = rx_buf[2];


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
	tx_buf[0] = SPI_message->send_confirm.start_byte;
	memcpy(&tx_buf[1], (void*)SPI_message->send_confirm.data, MAX_DATA_SIZE);
	tx_buf[1 + MAX_DATA_SIZE] = SPI_message->send_confirm.end_byte;
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

	memset(spi_frame_buffer,0,MAX_DATA_SIZE);
	memset(spi_receive_confirmation_buffer,0,BUFFER_RECEIVE_CONFIRMATION_SIZE);
	memset(spi_send_confirmation_buffer,0,BUFFER_SEND_CONFIRMATION_SIZE);

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
		default:
			SLAVE_STATE = IDLE_STATE;
			//Handle unknown command error
			break;
	}


}



//interupt called when SPI transmission is complete
// value sending confirmation will be handled in main loop
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	switch(SPI_state){
		case MESSAGE_RECEIVED:
			uint8_t error_code = copying_from_buffer_normal_frame(&SPI_message);
			preparing_response_for_master(&SPI_message,error_code);

			SPI_state = SENDING_CONFIRMATION;
			break;

		case CONFIRMATION_RECEIVED:
			copying_from_buffer_confirmation_frame(&SPI_message);
			HAL_SPI_Receive_DMA(hspi,spi_frame_buffer,BUFFER_FRAME_SIZE);
			break;

	}

}

