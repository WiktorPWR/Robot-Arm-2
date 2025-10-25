/*
 * SPI_functions.h
 *
 *  Created on: Oct 11, 2025
 *      Author: ostro
 */

#ifndef INC_SPI_FUNCTIONS_H_
#define INC_SPI_FUNCTIONS_H_

#include "data_templates.h"



extern uint8_t data_buffer[MAX_DATA_SIZE];
extern volatile uint8_t clear_spi_frame;

extern uint8_t spi_frame_buffer[BUFFER_FRAME_SIZE];
extern uint8_t spi_receive_confirmation_buffer[BUFFER_RECEIVE_CONFIRMATION_SIZE];
extern uint8_t spi_send_confirmation_buffer[BUFFER_SEND_CONFIRMATION_SIZE];

extern volatile struct message SPI_message;
extern volatile enum communication_states SPI_state;
extern volatile enum slave_state SLAVE_STATE;

/**
 * @brief Process a received SPI command message.
 *
 * @param SPI_message Pointer to the received SPI message structure.
 */
void process_received_command(struct message* SPI_message);

uint8_t copying_to_buffer_confirmation_frame(struct message* SPI_message);

#endif /* INC_SPI_FUNCTIONS_H_ */
