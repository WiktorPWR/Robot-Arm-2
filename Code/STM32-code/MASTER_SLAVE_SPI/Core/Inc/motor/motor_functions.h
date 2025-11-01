/*
 * motor_functions.h
 *
 *  Created on: Sep 14, 2025
 *      Author: ostro
 */

#ifndef INC_MOTOR_FUNCTIONS_H_
#define INC_MOTOR_FUNCTIONS_H_



uint8_t homming();

void motor_enable();

void motor_disable();

void motor_rotate_left();

void motor_rotate_right();

void motor_step_manual();

void set_speed(uint16_t speed);


#endif /* INC_MOTOR_FUNCTIONS_H_ */
