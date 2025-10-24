/*
 * SPI_functions.c
 *
 *  Created on: Oct 11, 2025
 *      Author: ostro
 */

#include "main.h"
#include "SPI_functions.h"

//interupt called when SPI transmission is complete
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

}

void new_function(){
	//your code here
}
void mariusz() {

}

void proccess_received_command(struct message* SPI_message){
	switch(SPI_message->message_frame.command){
	}
}
