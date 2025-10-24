/*
 * Communication_variables.h
 *
 *  Created on: Sep 5, 2025
 *      Author: ostro
 */

#ifndef INC_DATA_TEMPLATES_H_
#define INC_DATA_TEMPLATES_H_


/* ==========================================================
 *  CONSTANT DEFINITIONS
 * ========================================================== */

/**
 * @brief Frame delimiters used in communication.
 */
#define START_BYTE 0xAA
#define END_BYTE   0xBB

/**
 * @brief Maximum data payload size (in bytes) within a message frame.
 */
#define MAX_DATA_SIZE 20


/* ==========================================================
 *  COMMAND DATA LENGTHS
 * ========================================================== */

/**
 * @brief Expected data lengths for each command type.
 */
#define HOMMING_SIZE_OF_DATA          0
#define MOVING_SIZE_OF_DATA           4
#define STATUS_SIZE_OF_DATA           0
#define STOP_SIZE_OF_DATA             0
#define MOVEMENT_VALUES_SIZE_OF_DATA  8
#define DIAGNOSTICS_SIZE_OF_DATA      0


/* ==========================================================
 *  GLOBAL VARIABLES
 * ========================================================== */

/**
 * @brief Flag set when the SPI slave is not responding.
 */
volatile uint8_t SLAVE_DEAD = 0;

/**
 * @brief General-purpose buffer for received data.
 */
uint8_t data_buffer[MAX_DATA_SIZE];

/**
 * @brief Flag indicating that the current SPI frame should be cleared.
 */
volatile uint8_t clear_spi_frame = 0;


/* ==========================================================
 *  ENUMERATIONS
 * ========================================================== */

/**
 * @brief System operation states.
 */
enum states {
	IDLE_STATE,
	HOMMING_STATE,
	MOVING_STATE,
	ERROR_STATE,
	DIAGNOSTIC_STATE,
	MANUAL_CONTROL_STATE
};

/**
 * @brief Communication states used to manage SPI message flow.
 */
enum communication_states {
	MESSAGE_RECEIVED,        /*!< SPI frame was received and is ready to process */
	SENDING_CONFIRMATION,    /*!< Waiting to send confirmation frame */
	CONFIRMATION_RECEIVED    /*!< Confirmation frame received successfully */
};

/**
 * @brief Acknowledgment values exchanged between master and slave.
 */
enum acnowledgment_values {
	ACKNOWLEDGMENT_OK    = 0x01,
	ACKNOWLEDGMENT_ERROR = 0x00
};

/**
 * @brief Main communication commands recognized by the system.
 */
enum commands {
	START_HOMMING_COMMAND = 1,
	START_MOVING_COMMAND,
	GET_STATUS_COMMAND,
	STOP_COMMAND,
	CHANGE_MOVEMENT_VALUES,
	START_DIAGNOSTIC_COMMAND
};

/**
 * @brief Error codes returned by message parsing and processing functions.
 */
enum error_table {
	NO_ERROR,
	WRONG_START_BYTE,
	NO_SUCH_COMMAND_VALUE,
	WRONG_LENGTH_VALUE,
	WRONG_END_BYTE,
	ALLOCATION_ERROR,
	WRONG_CONFIRMATION_FRAME
};


/* ==========================================================
 *  DATA STRUCTURES
 * ========================================================== */

/**
 * @brief Data frame exchanged over SPI.
 */
struct message_frame {
	uint8_t start_byte;                  /*!< Frame start byte (always START_BYTE) */
	uint8_t command;                     /*!< Command identifier */
	uint8_t length;                      /*!< Length of valid data[] bytes */
	uint8_t data[MAX_DATA_SIZE];         /*!< Payload data */
	uint8_t end_byte;                    /*!< Frame end byte (always END_BYTE) */
};

/**
 * @brief Confirmation frame for acknowledging message reception.
 */
struct confirmation_frame {
	uint8_t start_byte;                  /*!< Start byte (usually START_BYTE) */
	uint8_t data;                        /*!< Acknowledgment value (OK/ERROR) */
	uint8_t end_byte;                    /*!< End byte (usually END_BYTE) */
};

/**
 * @brief Full SPI message container.
 */
struct message {
	uint8_t acknowledge_confirmation;    /*!< Set to 1 if acknowledgment received */
	struct message_frame frame;          /*!< Main data frame */
	struct confirmation_frame confirm;   /*!< Confirmation frame */
};


/* ==========================================================
 *  GLOBAL MESSAGE INSTANCE
 * ========================================================== */

/**
 * @brief Global SPI message structure used for communication handling.
 * @note Declared volatile because it may be modified in interrupt context.
 */
volatile struct message SPI_message;

/**
 * @brief Current state of SPI communication.
 * @note Declared volatile because it may be modified in interrupt context.
 */
volatile enum communication_states SPI_state;

#endif /* INC_DATA_TEMPLATES_H_ */
