/*	BASIC INTERRUPT VECTOR TABLE FOR STM8 devices
 *	Copyright (c) 2007 STMicroelectronics
 */
#include "stm8s_uart1.h"
#include "communication_variables.h"
//#include "stm8_interrupt_vector.h"

typedef void @far (*interrupt_handler_t)(void);

struct interrupt_vector {
	unsigned char interrupt_instruction;
	interrupt_handler_t interrupt_handler;
};

@far @interrupt void NonHandledInterrupt (void)
{
	/* in order to detect unexpected events during development, 
	   it is recommended to set a breakpoint on the following instruction
	*/
	return;
}

//My prototype
@far @interrupt void UART1_RX_IRQHandler(void)
{
	//recive data
	if(UART1_GetFlagStatus(UART1_FLAG_RXNE) == ENABLE && communication_master_flag != COMMUNICATION_MASTER_RECEIVE){		
		uint8_t data = UART1_ReceiveData8();
	
		//Quic process data
		if(data == START_BYTE){
			frame_element = START_BYTE;
			frame.start_byte = data;
		}else if(data == COMMAND_BYTE){
			frame_element = COMMAND_BYTE;
			frame.command = data;
		}else if(data == END_BYTE){
			frame_element = END_BYTE;
			frame.end_byte = data;
			communication_master_flag = COMMUNICATION_MASTER_RECEIVE;
		}else{
			communication_master_flag = COMMUNICATION_MASTER_ERROR;
			frame_element = IDLE;
			frame.start_byte = 0;
			frame.command = 0;
			frame.end_byte = 0;
		}

		//Dont forget to reset flag 
		UART1_ClearFlag(UART1_FLAG_RXNE);
	}

}




extern void _stext();     /* startup routine */

struct interrupt_vector const _vectab[] = {
	{0x82, (interrupt_handler_t)_stext}, /* reset */
	{0x82, NonHandledInterrupt}, /* trap  */
	{0x82, NonHandledInterrupt}, /* irq0  */
	{0x82, NonHandledInterrupt}, /* irq1  */
	{0x82, NonHandledInterrupt}, /* irq2  */
	{0x82, NonHandledInterrupt}, /* irq3  */
	{0x82, NonHandledInterrupt}, /* irq4  */
	{0x82, NonHandledInterrupt}, /* irq5  */
	{0x82, NonHandledInterrupt}, /* irq6  */
	{0x82, NonHandledInterrupt}, /* irq7  */
	{0x82, NonHandledInterrupt}, /* irq8  */
	{0x82, NonHandledInterrupt}, /* irq9  */
	{0x82, NonHandledInterrupt}, /* irq10 */
	{0x82, NonHandledInterrupt}, /* irq11 */
	{0x82, NonHandledInterrupt}, /* irq12 */
	{0x82, NonHandledInterrupt}, /* irq13 */
	{0x82, NonHandledInterrupt}, /* irq14 */
	{0x82, NonHandledInterrupt}, /* irq15 */
	{0x82, NonHandledInterrupt}, /* irq16 */
	{0x82, UART1_RX_IRQHandler}, /* irq17 */
	{0x82, NonHandledInterrupt}, /* irq18 */
	{0x82, NonHandledInterrupt}, /* irq19 */
	{0x82, NonHandledInterrupt}, /* irq20 */
	{0x82, NonHandledInterrupt}, /* irq21 */
	{0x82, NonHandledInterrupt}, /* irq22 */
	{0x82, NonHandledInterrupt}, /* irq23 */
	{0x82, NonHandledInterrupt}, /* irq24 */
	{0x82, NonHandledInterrupt}, /* irq25 */
	{0x82, NonHandledInterrupt}, /* irq26 */
	{0x82, NonHandledInterrupt}, /* irq27 */
	{0x82, NonHandledInterrupt}, /* irq28 */
	{0x82, NonHandledInterrupt}, /* irq29 */
};
