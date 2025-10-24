/*
 * Communication_variables.h
 *
 *  Created on: Sep 5, 2025
 *      Author: ostro
 */

#ifndef INC_DATA_TEMPLATES_H_
#define INC_DATA_TEMPLATES_H_

enum states{
	IDLE_STATE,
	HOMMING_STATE,
	MOVING_STATE,
	ERROR_STATE,
	DIAGNOSTIC_STATE,
	MANUAL_CONTROL_STATE,
};

struct message{
	struct message_frame{
		uint8_t start_byte;
		uint8_t command;
		uint8_t length;
		uint8_t data[56];//this value is set by 60 is isze of buffer and we just decrese by 4 bajts
		uint8_t end_byte;
	};

	struct confirmation_frame{
		uint8_t start_byte;
		uint8_t data[1];
		uint8_t end_byte;
	};

	uint8_t acknowledge_confirmation;//if slave send back to master all data that command was received
};

uint8_t new_message_available;//flag to indicate that new message is received

struct diagnostic_summary{
	uint8_t AS5600;
	uint8_t incremental_enkoder;
	uint8_t	motor;
	uint8_t endstop;
};

#endif /* INC_DATA_TEMPLATES_H_ */
