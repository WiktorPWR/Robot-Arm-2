#include "motor_setup_functions.h"
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>


static const uint16_t DUTY_CYCLE = 50;
static const uint16_t START_FREQUENCY = 1000; // 1 kHz

#define CCR_CALCULATION(frequency, timer) ((HAL_RCC_GetPCLK1Freq() / (frequency * (timer->Instance->PSC + 1))) - 1)

//This is functionality for state machine of the motor
typedef enum MOTOR_STATE{
    MOTOR_STOP,
    MOTOR_RUNNING,
    MOTOR_ERROR
};

static MOTOR_STATE motor_state = MOTOR_STOP;

MOTOR_STATE motor_state_getter(void){
    return motor_state;
};

static void motor_state_setter(MOTOR_STATE new_motor_state){
    motor_state = new_motor_state;
}


//This part is for setting direction of the motor 

typedef enum MOTOR_DIRECTION{
    NONE,
    LEFT,
    RIGTH
};

MOTOR_DIRECTION motor_direction = NONE;

MOTOR_DIRECTION motor_direction_getter(void){
    return motor_direction;
};

void motor_direction_setter(MOTOR_DIRECTION new_motor_direction){
    motor_direction = new_motor_direction;
}

//This part is for enable motor or not

typedef enum MOTOR_ENABLE{
    NONE,
    DISABLED,
    ENABLED
};

MOTOR_ENABLE motor_enable = DISABLED;

MOTOR_ENABLE motor_enable_getter(void){
    return motor_enable;
};

void motor_enable_setter(MOTOR_ENABLE new_motor_enable){
    motor_enable = new_motor_enable;
}



//This part is for setting speed of the motor
HAL_StatusTypeDef motor_speed_setter(uint16_t frequency, TIM_HandleTypeDef *timer){

    // If frequency is 0, stop the motor
    if(frequency == 0){
        //We stop the timer to stop the motor
        if(HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1) != HAL_OK){
            motor_state_setter(MOTOR_ERROR);
            return HAL_ERROR;
        }
        // Set the motor state to stop
        motor_state_setter(MOTOR_STOP);
        return HAL_OK;
    }

    // if motor is just starting, start the timer
    if(motor_state_getter() == MOTOR_STOP){ // Check if the timer is enabled)

        //before calcualtion we set arr to 0 for better calculation of the prescaler
        timer->Instance->ARR = 0;

        //Then we calculate the prescaler value based on the desired frequency and the timer clock frequency
        uint32_t APB1_CLK = HAL_RCC_GetPCLK1Freq();
        timer->Instance->PSC = ((APB1_CLK / START_FREQUENCY) - 1); // Calculate prescaler for 1 kHz frequency

        //When we have this set we can start PWM signal generation
        if(HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK){
            motor_state_setter(MOTOR_ERROR);
            return HAL_ERROR;
        }
        
        // Set the motor state to running
        motor_state_setter(MOTOR_RUNNING);

    }

    // if motor is running then we just update the arr value to change the frequency
    if(motor_state_getter() == MOTOR_RUNNING){
        uint32_t APB1_CLK = HAL_RCC_GetPCLK1Freq();
        timer->Instance->ARR = (APB1_CLK / (frequency * (timer->Instance->PSC + 1))) - 1; // Calculate ARR value for desired frequency
    }

    return HAL_OK;
}