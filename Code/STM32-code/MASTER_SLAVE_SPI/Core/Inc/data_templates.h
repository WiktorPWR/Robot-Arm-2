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

/**
 * @brief Identifier for this slave device in the communication protocol.
 */

#define SLAVE_ID 0x01


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
#define ACCEPT_CONFIRMATION_SIZE_OF_DATA 0

/* ==========================================================
 *  GLOBAL VARIABLES
 * ========================================================== */

/**
 * @brief Buffer sizes for SPI communication.
 */
#define BUFFER_FRAME_SIZE 25
#define BUFFER_RECEIVE_CONFIRMATION_SIZE 4


/* ==========================================================
 *  ENUMERATIONS
 * ========================================================== */

/**
 * @brief System operation states.
 */
enum slave_state {
	IDLE_STATE,
	HOMMING_STATE,
	MOVING_STATE,
	ERROR_STATE,
	STOP_STATE,
	DIAGNOSTIC_STATE,
	MANUAL_CONTROL_STATE,
	SENDING_STATE,
	CHAGING_MOVEMENT_VALUES_STATE,
};

/**
 * @brief Communication states used to manage SPI message flow.
 */
enum communication_states {
	MESSAGE_RECEIVED,        /*!< SPI frame was received and is ready to process */
	SENDING_CONFIRMATION,    /*!< Waiting to send confirmation frame */
	CONFIRMATION_RECEIVED,    /*!< Confirmation frame received successfully */
	PROCES_CONFRMATION
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
	START_DIAGNOSTIC_COMMAND,
	ACCEPT_CONFIRMATION_COMMAND
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
	WRONG_START_BYTE_CONFIRMATION,
	WRONG_END_BYTE_CONFIRMATION,
	BEFOR_USING_ANY_NEW_COMMAND_RESET_SLAVE
};


/* ==========================================================
 *  DATA STRUCTURES
 * ========================================================== */

/**
 * @brief Data frame exchanged over SPI.
 */
struct message_frame {
	uint8_t start_byte;                  /*!< Frame start byte (always START_BYTE) */
	uint8_t slave_id;					/*!< Identifier for the slave device */
	uint8_t command;                     /*!< Command identifier */
	uint8_t length;                      /*!< Length of valid data[] bytes */
	uint8_t data[MAX_DATA_SIZE];         /*!< Payload data */
	uint8_t end_byte;                    /*!< Frame end byte (always END_BYTE) */
};

/**
 * @brief Confirmation frame for acknowledging message reception.
 */
struct confirmation_receive_frame {
	uint8_t start_byte;                  /*!< Start byte (usually START_BYTE) */
	uint8_t slave_id;					/*!< Identifier for the slave device */
	uint8_t data;                        /*!< Acknowledgment value (OK/ERROR) */
	uint8_t end_byte;                    /*!< End byte (usually END_BYTE) */
};


/**
 * @brief Full SPI message container.
 */
struct message {
	uint8_t acknowledge_confirmation;    /*!< Set to 1 if acknowledgment received */
	struct message_frame frame;          /*!< Main data frame */
	struct confirmation_receive_frame receive_confirm; /*!< Confirmation frame received */
	struct message_frame send_confirm; /*!< Confirmation frame to send */
};


/* ==========================================================
 *  GLOBAL MESSAGE INSTANCE
 * ========================================================== */

#endif /* INC_DATA_TEMPLATES_H_ */
