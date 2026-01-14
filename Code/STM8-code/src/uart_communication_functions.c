#include "uart_communication_functions.h"


const uint32_t BaudRate = 9600;
const UART1_WordLength_TypeDef WordLength = UART1_WORDLENGTH_8D;
const UART1_StopBits_TypeDef StopBits = UART1_STOPBITS_1;
const UART1_Parity_TypeDef Parity = UART1_PARITY_EVEN;
const UART1_SyncMode_TypeDef SyncMode = UART1_SYNCMODE_CLOCK_DISABLE;
const UART1_Mode_TypeDef Mode = UART1_MODE_TXRX_ENABLE;

const UART1_IT_TypeDef UART_IT = UART1_IT_RXNE;



void uart_setting_up(void)
{
    // Reset ITC i UART
    ITC_DeInit();
    UART1_DeInit();

    // Inicjalizacja UART z u�yciem zmiennych
    UART1_Init(BaudRate, WordLength, StopBits, Parity, SyncMode, Mode);

    // W��cz przerwanie RX
    UART1_ITConfig(UART_IT, ENABLE);

    // W��cz tryb half-duplex je�li potrzebny
    UART1_HalfDuplexCmd(ENABLE);

    // W��cz UART
    UART1_Cmd(ENABLE);
}


void send_data(uint8_t* data, uint8_t length)
{
		uint8_t i = 0;
    for( i; i < length; i++){
        // Czekaj na gotowo���� do wysy��ania
        while(UART1_GetFlagStatus(UART1_FLAG_TXE) == DISABLE);
        
        // Wy��lij bajt
        UART1_SendData8(data[i]);
    }
}

void send_uint16(uint16_t value)
{
    uint8_t high = (value >> 8) & 0xFF; // wyższy bajt
    uint8_t low  = value & 0xFF;        // niższy bajt

    send_data(&high, 1);
    send_data(&low, 1);
}


void proces_master_command(void)
{
    switch(master_commands)
    {
        case READ_POSITION:
            send_uint16(read_buffer.position);
            break;

        case READ_SPEED:
            send_uint16(read_buffer.speed);
            break;

        case READ_ACCEL:
            send_uint16(read_buffer.accel);
            break;

        default:
            // brak komendy lub nieznana komenda
            break;
        
    }
    communication_master_flag = COMMUNICATION_MASTER_IDLE;
    frame_element = IDLE;
    frame.start_byte = 0;
    frame.command = 0;
    frame.end_byte = 0;
}