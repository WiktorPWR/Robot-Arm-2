/* MAIN.C file
 * 
 * Copyright (c) 2002-2005 STMicroelectronics
 */
#include "stm8s.h"
#include "stm8s_uart1.h"
//#include "stm8_interrupt_vector.h"
#include "communication_variables.h"
#include "uart_communication_functions.h"
#include "stm8s_iwdg.h"

//ok watchdog we need to do some stuff





main(){

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(0xFF);
    IWDG_ReloadCounter();
    

    uart_setting_up();

    enableInterrupts();

    IWDG_Enable();
    
    {
        while (1)
        {
            IWDG_ReloadCounter();
            if(communication_master_flag == COMMUNICATION_MASTER_RECEIVE){
                proces_master_command();
            }
        }
    }
}