/*
 * motor_functions.c
 *
 *  Created on: Sep 14, 2025
 *      Author: ostro
 */
#include "main.h"
#include "endstop.h"


extern TIM_HandleTypeDef htim4;

const uint16_t ARR_VALUE = 65535;
const uint32_t CLOCK_VALUE = 8000000;// 8 000 000
const uint8_t  MICROSTEPPING = 16;
const float  DUTY_CYCLE = 0.50;
const uint16_t MINIMAL_SPEED = 5;
const uint16_t MAXIMAL_SPEED = 400;

uint16_t counter_ticks = 0;
uint8_t after_homming = 0;

volatile uint8_t endstop_state;

void motor_enable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
}

void motor_disable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
}

void motor_rotate_left(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
}

void motor_rotate_right(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
}

void motor_step_manual(){
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_SET);
	HAL_Delay(200);//200 ms
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_RESET);
}


void motor_stop(){
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

static uint8_t is_pwm_active_ch1(void) {
    // 1. Timer włączony?
    if ((htim4.Instance->CR1 & TIM_CR1_CEN) == 0) {
        return 0;
    }

    // 2. Kanał CH1 włączony?
    if ((htim4.Instance->CCER & TIM_CCER_CC1E) == 0) {
        return 0;
    }

    // 3. Tryb ustawiony na PWM (OC1M = 110 lub 111)
    uint32_t oc1m = (htim4.Instance->CCMR1 >> 4) & 0x7;
    if (oc1m < 6) {
        return 0;
    }

    return 1; // PWM aktywny
}


//predkosc bedzie podawana w obr/minute bo lubie ta jednostke
// maksymalna predkosc to niech bedzie np 300 obrotow na minute
// a minimalna to niech bedzie sobie nawet to 10 obrotow na minute
// Funkcja pomocnicza do sprawdzania czy PWM działa na TIM4_CH1
//funckja ponizej ustawia predkosc walu silnika nie walu przekładni
void set_speed(int16_t speed) {

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


uint8_t homming(){
	while(endstop_state != 1){
		set_speed(MINIMAL_SPEED);
	}
	motor_stop();
	counter_ticks = 0;
	after_homming = 1;
	return 0;
}


uint8_t move_via_angle(){

};


