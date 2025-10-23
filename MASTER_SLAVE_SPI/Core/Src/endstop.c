/*
 * endstop.c
 *
 *  Created on: Sep 26, 2025
 *      Author: ostro
 */

#include "endstop.h"
#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ENDSTOP_Pin){
		if(HAL_GPIO_ReadPin(GPIOA, ENDSTOP_Pin)){
			endstop_state = 1;
		}else{
			endstop_state = 0;
		}
	}
}
