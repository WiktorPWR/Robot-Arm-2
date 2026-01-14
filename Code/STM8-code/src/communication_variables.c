#include "communication_variables.h"

volatile REGISTERS write_buffer;
volatile REGISTERS read_buffer;

volatile FRAME_ELEMENTS frame_element = IDLE;
volatile MASTER_COMMANDS master_commands = NO_COMMAND;

volatile FRAME frame = {0};

volatile COMMUNICATION_MASTER_FLAG communication_master_flag = COMMUNICATION_MASTER_IDLE;