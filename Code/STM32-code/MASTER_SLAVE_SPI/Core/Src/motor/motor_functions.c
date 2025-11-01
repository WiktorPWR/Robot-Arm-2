/*
 * motor_functions.c
 *
 *  Created on: Sep 14, 2025
 *      Author: ostro
 */
#include "main.h"
#include "endstop.h"
#include "motor/motor_data.h"
#include "motor/motor_functions.h"
#include "motor/S_curve_mathematic.h"

uint16_t MINIMAL_SPEED =  5;
uint16_t MAXIMAL_SPEED = 400;
uint16_t MAX_ACCELERATION = 50;
uint16_t MAX_JERK = 10;

enum Rotation_Direction rotation_direction;
enum Motor_State motor_state;
enum Homming_State homming_state = NOT_HOMMED;
enum IS_PWM_ACTIVE pwm_state;


extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

volatile uint16_t actual_position;
volatile uint8_t endstop_state;


/**
 * @brief  Enables the motor driver (sets ENABLE pin high).
 * @note   Sets the motor state to MOTOR_ENABLE.
 */
void motor_enable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
	motor_state = MOTOR_ENABLE;
}

/**
 * @brief  Disables the motor driver (sets ENABLE pin low).
 * @note   Sets the motor state to MOTOR_DISABLE.
 */
void motor_disable(){
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
	motor_state = MOTOR_DISABLE;
}


/**
 * @brief  Sets rotation direction to LEFT.
 */
void motor_rotate_left(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	rotation_direction = LEFT;
}

/**
 * @brief  Sets rotation direction to RIGHT.
 */
void motor_rotate_right(){
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	rotation_direction = RIGHT;
}


/**
 * @brief  Generates a single manual step pulse on the PULL_MANUAL pin.
 * @note   Useful for manual fine-positioning or debugging.
 */
void motor_step_manual(){
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_SET);
	HAL_Delay(10);//10 ms
	HAL_GPIO_WritePin(PULL_MANUAL_GPIO_Port, PULL_MANUAL_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Stops PWM output for motor motion (disables channel 1).
 */
void motor_stop(){
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}


/**
 * @brief  Checks if PWM output on TIM4 Channel 1 is active.
 * @retval PWM_ACTIVE if timer and channel are configured and enabled.
 * @retval TIMER_NOT_ACTIVE, CANAL_NOT_ACTIVE, or SETUP_NOT_ACTIVE otherwise.
 */
static uint8_t is_pwm_active_ch1(void) {
    // 1. Timer enabled?
    if ((htim4.Instance->CR1 & TIM_CR1_CEN) == 0)
        return TIMER_NOT_ACTIVE;

    // 2. Channel enabled?
    if ((htim4.Instance->CCER & TIM_CCER_CC1E) == 0)
        return CANAL_NOT_ACTIVE;

    // 3. PWM mode set?
    uint32_t oc1m = (htim4.Instance->CCMR1 >> 4) & 0x7;
    if (oc1m < 6)
        return SETUP_NOT_ACTIVE;

    return PWM_ACTIVE;
}


/**
 * @brief  Configures the PWM timer to generate step pulses at a frequency
 *         corresponding to the desired motor speed (steps per second).
 *
 * @param  speed_steps_per_s  Desired pulse frequency in steps per second.
 *
 * @note
 * - If the speed is set to zero, PWM is stopped immediately.
 * - ARR and PSC are recalculated dynamically to achieve the target frequency.
 * - 50% duty cycle ensures clean step pulses for the stepper driver.
 */
void set_speed(uint16_t speed_steps_per_s)
{
    // --- 1. Handle case when motor should stop ---
    // If requested speed = 0 → stop PWM output entirely.
    if (speed_steps_per_s == 0)
    {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        return;
    }

    // --- 2. Calculate initial ARR value (assuming prescaler = 0) ---
    // Formula: ARR = (CLOCK_VALUE / Fpwm) - 1
    // Example: 8 MHz / 2000 steps/s = 4000 → ARR = 3999
    uint32_t arr_value = (CLOCK_VALUE / speed_steps_per_s) - 1;

    // --- 3. Adjust prescaler dynamically if ARR exceeds 16-bit limit ---
    // 16-bit timers can only count up to 65535.
    // If ARR > 65535, increase prescaler until ARR fits in range.
    uint16_t prescaler = 0;
    while (arr_value > 0xFFFF)
    {
        prescaler++;
        arr_value = (CLOCK_VALUE / ((prescaler + 1) * speed_steps_per_s)) - 1;
    }

    // --- 4. Apply calculated values to timer registers ---
    // PSC  → divides the timer input clock
    // ARR  → defines PWM period (frequency)
    // CCR1 → defines duty cycle (50%)
    htim4.Instance->PSC  = prescaler;
    htim4.Instance->ARR  = (uint16_t)arr_value;
    htim4.Instance->CCR1 = (uint16_t)(0.5f * arr_value); // 50% high / 50% low

    // --- 5. Check if PWM channel is already active ---
    // If not, start PWM generation on TIM4 Channel 1.
    if (!is_pwm_active_ch1())
    {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    }

    // --- 6. Function complete ---
    // Timer now outputs a square wave signal with frequency = speed_steps_per_s.
    // Each rising edge corresponds to one step pulse for the driver.
}



/**
 * @brief  Updates minimal and maximal movement speed parameters.
 * @param  min_speed: Minimal speed value.
 * @param  max_speed: Maximal speed value.
 */
void change_movement_parameters(uint16_t min_speed, uint16_t max_speed, uint16_t max_acceleraiotn, uint16_t max_jerk ){
	MINIMAL_SPEED = min_speed;
	MAXIMAL_SPEED = max_speed;
	MAX_ACCELERATION = max_acceleraiotn;
	MAX_JERK = max_jerk;
}


/**
 * @brief  Performs homing procedure until endstop is triggered.
 * @retval HOMMED if successful.
 * @note   Runs motor at MINIMAL_SPEED until endstop_state == 1.
 */
uint8_t homming(){
	while(endstop_state != 1){
		set_speed(MINIMAL_SPEED);
	}
	motor_stop();

	return HOMMED;
}

/**
 * @brief  Generates S-curve motion trajectory based on target position.
 * @param  target_position: Desired motor target position (degrees or steps).
 * @note   Configures motion parameters and fills trajectory buffer.
 */
void calculating_speed_profile(float target_position){
	config.max_velocity = MAXIMAL_SPEED;
	config.max_acceleration = MAX_ACCELERATION;
	config.max_jerk = MAX_JERK;
	config.interpolation_points = 100;

	// Configure motor (example: 3:1 gear ratio, 16 microsteps, 200 steps/rev)
	set_motor_config(GEAR_RATIO, MICROSTEPPING, STEPS_PER_REVOLUTION);

	generate_scurve_trajectory(target_position);

}


/**
 * @brief  Moves the motor by a specified angular distance using S-curve profile.
 * @param  angle: Target position (in degrees or equivalent units).
 * @retval HOMMED, NOT_HOMMED, or PWM state error codes.
 * @note   Executes velocity profile and fine-positioning correction.
 */
uint8_t move_via_angle(float angle)
{
    // Check if homing was done
    if (homming_state != HOMMED)
        return NOT_HOMMED;

    // Check PWM state
    uint8_t pwm_status = is_pwm_active_ch1();
    if (pwm_status != PWM_ACTIVE)
        return pwm_status;

    // Determine rotation direction
    if (angle > actual_position)
        rotation_direction = LEFT;
    else
        rotation_direction = RIGHT;

    // Enable motor if needed
    if (motor_state == MOTOR_DISABLE)
        motor_enable();

    // Generate motion profile
    calculating_speed_profile(angle);

    // Execute motion
    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = 0;
    uint8_t actual_value = 0;

    for (uint16_t i = 0; i < buffer_count; i++) {
        current_time = HAL_GetTick();
        while (trajectory_buffer[i].time_ms >= (current_time - start_time)) {
            if (actual_value == 0) {
                set_speed(trajectory_buffer[i].velocity_raw);
                actual_value = 1;
            }
        }
        actual_value = 0;
    }

    // Fine positioning phase (creep move)
    while (actual_position < angle + PERMITTED_POSITION_DEVIATION_VALUE &&
           actual_position > angle - PERMITTED_POSITION_DEVIATION_VALUE) {
        motor_step_manual();
    }

    // Overshoot slightly for backlash compensation
    for (uint8_t i = 0; i < ADDITIONAL_STEP; i++) {
        motor_step_manual();
    }

    return MOVEMENT_DONE;
}

