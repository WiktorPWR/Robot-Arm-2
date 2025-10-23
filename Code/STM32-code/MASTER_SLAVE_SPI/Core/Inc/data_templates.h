/*
 * Communication_variables.h
 *
 *  Created on: Sep 5, 2025
 *      Author: ostro
 */

#ifndef INC_DATA_TEMPLATES_H_
#define INC_DATA_TEMPLATES_H_

enum slave_state{
	READY,// READY = R
	BUSY//BUSY = B
};

enum master_command{
	STATUS,
	MOVE,
	HOMMING,
	DIAGNOSTICS,
	NONE
};

struct message_frame{
	uint8_t start_byte;
	uint8_t command;
	uint8_t length;
	uint8_t data[56];//this value is set by 60 is isze of buffer and we just decrese by 4 bajts
	uint8_t end_byte;
};


struct diagnostic_summary{
	uint8_t AS5600;
	uint8_t incremental_enkoder;
	uint8_t	motor;
	uint8_t endstop;
};

#endif /* INC_DATA_TEMPLATES_H_ */
