/*
 * Communication_variables.h
 *
 *  Created on: Sep 5, 2025
 *      Author: ostro
 */

#ifndef INC_DATA_TEMPLATES_H_
#define INC_DATA_TEMPLATES_H_

#define START_BYTE 0xAA
#define END_BYTE 0xBB

#define MAX_DATA_SIZE 20

#define HOMMING_SIZE_OF_DATA  0
#define MOVING_SIZE_OF_DATA  4
#define STATUS_SIZE_OF_DATA  0
#define STOP_SIZE_OF_DATA  0
#define MOVEMENT_VALUES_SIZE_OF_DATA  8
#define DIAGNOSTICS_SIZE_OF_DATA  0


enum states{
	IDLE_STATE,
	HOMMING_STATE,
	MOVING_STATE,
	ERROR_STATE,
	DIAGNOSTIC_STATE,
	MANUAL_CONTROL_STATE
};


enum commands{
	START_HOMMING_COMMAND = 1,
	START_MOVING_COMMAND ,
	GET_STATUS_COMMAND ,
	STOP_COMMAND ,
	CHANGE_MOVEMENT_VALUES,
	START_DIAGNOSTIC_COMMAND
};


struct message_frame{
	uint8_t start_byte;
	uint8_t command;
	uint8_t length;
	uint8_t *data;//this value is set by 60 is isze of buffer and we just decrese by 4 bajts
	uint8_t end_byte;
};

struct confirmation_frame{
	uint8_t start_byte;
	uint8_t data;
	uint8_t end_byte;
};

struct message {
    struct message_frame frame;          // pole typu message_frame
    struct confirmation_frame confirm;   // pole typu confirmation_frame
    uint8_t acknowledge_confirmation;
};

uint8_t new_message_available;//flag to indicate that new message is received

enum error_table{
	NO_ERROR,
	WRONG_START_BYTE,
	NO_SUCH_COMMAND_VALUE,
	WRONG_LENGTH_VALUE,
	WRONG_END_BYTE
};

#endif /* INC_DATA_TEMPLATES_H_ */
