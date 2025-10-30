/*
 * motor_functions.c
 *
 *  Created on: Sep 14, 2025
 *      Author: ostro
 */
#include "main.h"
#include "endstop.h"
#include "motor/motor_functions.h"


enum Rotation_Direction rotation_direction;
enum Motor_State motor_state;
enum Homming_State homming_state = NOT_HOMMED;
enum IS_PWM_ACTIVE pwm_state;

uint8_t speed_profile[1000] = {0};

extern TIM_HandleTypeDef htim4;


volatile uint16_t actual_position;
volatile uint8_t endstop_state;

void motor_enable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	motor_state = ENABLED;
}

void motor_disable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
	motor_state = DISABLED;
}

void motor_rotate_left(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	rotation_direction = LEFT;
}

void motor_rotate_right(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	rotation_direction = RIGHT;
}

void motor_step_manual(){
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_SET);
	HAL_Delay(200);//200 ms
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_RESET);
}


void motor_stop(){
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	motor_state = STOPPED;
}

static uint8_t is_pwm_active_ch1(void) {
    // 1. Timer włączony?
    if ((htim4.Instance->CR1 & TIM_CR1_CEN) == 0) {
        return TIMER_NOT_ACTIVE;
    }

    // 2. Kanał CH1 włączony?
    if ((htim4.Instance->CCER & TIM_CCER_CC1E) == 0) {
        return CANAL_NOT_ACTIVE;
    }

    // 3. Tryb ustawiony na PWM (OC1M = 110 lub 111)
    uint32_t oc1m = (htim4.Instance->CCMR1 >> 4) & 0x7;
    if (oc1m < 6) {
        return SETUP_NOT_ACTIVE;
    }

    return PWM_ACTIVE; // PWM aktywny
}


//predkosc bedzie podawana w obr/minute bo lubie ta jednostke
// maksymalna predkosc to niech bedzie np 300 obrotow na minute
// a minimalna to niech bedzie sobie nawet to 10 obrotow na minute
// Funkcja pomocnicza do sprawdzania czy PWM działa na TIM4_CH1
//funckja ponizej ustawia predkosc walu silnika nie walu przekładni
void set_speed(int16_t speed) {
	motor_state = MOTOR_RUNNING;

    // Fpwm = Fclk / [(ARR+1)*(PSC+1)]
    // DutyCycle[%] = CCRx/ARR
    // timer base clock = 8MHz

    speed = speed / 60; // zamiana RPM → RPS
    const uint32_t Fpwm = speed * MICROSTEPPING * 200; // target frequency

    htim4.Instance->PSC  = 0; // prescaler = 0
    htim4.Instance->ARR  = (uint16_t)((CLOCK_VALUE / Fpwm) - 1);
    htim4.Instance->CCR1 = (uint16_t)(DUTY_CYCLE * htim4.Instance->ARR);

    // sprawdź czy PWM jest aktywny, jeśli nie — uruchom
    if (!is_pwm_active_ch1()) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    }
}

void change_movement_parameters(uint16_t min_speed, uint16_t max_speed){
	MINIMAL_SPEED = min_speed;
	MAXIMAL_SPEED = max_speed;
}

uint8_t move_via_angle(int16_t angle){
	if(after_homming == 0){
		return 1;
	}
	int32_t steps = (int32_t)((angle / 360.0) * MICROSTEPPING * 200);
	if(steps > 0){
		motor_rotate_right();
	}else if(steps < 0){
		motor_rotate_left();
		steps = -steps;
	}else{
		return 0;
	}
	set_speed(MAXIMAL_SPEED);
	uint32_t target_ticks = counter_ticks + steps;
	while(counter_ticks < target_ticks){
		//czekaj
	}
	motor_stop();
	return 0;
}


uint8_t homming(){
	while(endstop_state != 1){
		set_speed(MINIMAL_SPEED);
	}
	motor_stop();
	counter_ticks = 0;
	after_homming = 1;
	return HOMMED;
}

void calculatiing_speed_profile(float target_position){
	//to be implemented
}


uint8_t move_via_angle(float angle){

	//check if hommed
	if(homming_state != HOMMED){
		return NOT_HOMMED;
	}

	//check if PWM is active
	uint8_t pwm_status = is_pwm_active_ch1();
	if(pwm_status != PWM_ACTIVE){
		return pwm_status;
	}

	//Select direction of motor spin
	if(angle > actual_position){
		rotation_direction = LEFT;
	}
	else{
		rotation_direction = RIGHT;
	}

	//enable motor if its not active
	if(motor_state == DISABLED){
		motor_enable();
	}



};


