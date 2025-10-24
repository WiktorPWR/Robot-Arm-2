/*
 * SPI_functions.c
 *
 *  Created on: Oct 11, 2025
 *      Author: ostro
 */

#include "main.h"
#include "SPI_functions.h"

void proccess_received_command(struct message* SPI_message){
	switch(SPI_message->message_frame.command){
		case IDLE_STATE:
			//handle idle state command
			break;
		case HOMMING_STATE:
			//handle homming state command
			break;
		case MOVING_STATE:
			//handle moving state command
			 break;
		case ERROR_STATE:
			//handle error state command
			break;
		case DIAGNOSTIC_STATE:
			//handle diagnostic state command
			break;
		case MANUAL_CONTROL_STATE:
			//handle manual control state command
			break;
	}
}

uint8_t chec_message_frame(struct message* SPI_message){
	//Checl if start byte i correct
	if(SPI_message->frame.start_byte != START_BYTE){
		return WRONG_START_BYTE;
	}
	//Check if command value is correct
	switch(SPI_message->frame.command){
		case START_HOMMING_COMMAND:
		case START_MOVING_COMMAND:
		case STOP_COMMAND:
		case START_DIAGNOSTIC_COMMAND:
		case GET_STATUS_COMMAND:
			break;
		default:
			return NO_SUCH_COMMAND_VALUE;
	}

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
	}

	//Check end byte
	if(SPI_message->frame.end_byte != END_BYTE){
		return WRONG_END_BYTE;
	}

	//If all checks are passed return no error
	return NO_ERROR;
}


//interupt called when SPI transmission is complete
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	struct message SPI_message;


	//proccess_received_command(&SPI_message);
}

