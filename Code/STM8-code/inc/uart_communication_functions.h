#ifndef UART_COMMUNICATION_FUNCTIONS_H
#define UART_COMMUNICATION_FUNCTIONS_H

#include "stm8s_uart1.h"
#include "stm8s.h"
#include "communication_variables.h"

extern const uint32_t BaudRate;
extern const UART1_WordLength_TypeDef WordLength;
extern const UART1_StopBits_TypeDef StopBits;
extern const UART1_Parity_TypeDef Parity;
extern const UART1_SyncMode_TypeDef SyncMode;
extern const UART1_Mode_TypeDef Mode;
extern const UART1_IT_TypeDef UART_IT;

void uart_setting_up(void);

void send_data(uint8_t* data, uint8_t length);

void send_uint16(uint16_t value);

void proces_master_command(void);



#endif // UART_COMMUNICATION_FUNCTIONS_H