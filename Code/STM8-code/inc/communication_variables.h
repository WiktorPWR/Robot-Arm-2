#ifndef COMMUNICATION_VARIABLES
#define COMMUNICATION_VARIABLES

#include "stm8s.h"

// Buffers
typedef struct {
    uint16_t position;
    uint16_t speed;
    uint16_t accel;
} REGISTERS;

extern volatile REGISTERS write_buffer;
extern volatile REGISTERS read_buffer;

// Frame elements
typedef enum {
    IDLE = 0x00,
    START_BYTE = 0x01,
    COMMAND_BYTE = 0x02,
    END_BYTE = 0x03
} FRAME_ELEMENTS;

extern volatile FRAME_ELEMENTS frame_element;

// Master commands
typedef enum {
    NO_COMMAND = 0x00,
    READ_POSITION = 0x11,
    READ_SPEED = 0x22,
    READ_ACCEL = 0x33
} MASTER_COMMANDS;

extern volatile MASTER_COMMANDS master_commands;

// Frame structure
typedef struct {
    uint8_t start_byte;
    uint8_t command;
    uint8_t end_byte;
} FRAME;

extern volatile FRAME frame;

// Communication flags
typedef enum {
    COMMUNICATION_MASTER_IDLE = 0x00,
    COMMUNICATION_MASTER_ERROR = 0xFF,
    COMMUNICATION_MASTER_RECEIVE = 0x01
} COMMUNICATION_MASTER_FLAG;

extern volatile COMMUNICATION_MASTER_FLAG communication_master_flag;

#endif
