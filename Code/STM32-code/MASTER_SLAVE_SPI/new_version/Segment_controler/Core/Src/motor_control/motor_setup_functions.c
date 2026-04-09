#include "motor_setup_functions.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>


static const uint16_t DUTY_CYCLE = 50;
static const uint16_t START_FREQUENCY = 1000; // 1 kHz

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

static MOTOR_STATE motor_state_setter(MOTOR_STATE new_motor_state){
    motor_state = new_motor_state;
    return motor_state;
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

MOTOR_DIRECTION motor_direction_setter(MOTOR_DIRECTION new_motor_direction){
    motor_direction = new_motor_direction;
    return motor_direction;
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

MOTOR_ENABLE motor_enable_setter(MOTOR_ENABLE new_motor_enable){
    motor_enable = new_motor_enable;
    return motor_enable;
}

//This function always set out value to 50% duty cycle, so we can change the frequency without changing the duty cycle
static inline void DUTY_CYCLE_CALCULATION(TIM_HandleTypeDef *timer){
    timer->Instance->CCR1 = (timer->Instance->ARR * DUTY_CYCLE / 100); // Calculate CCR value for desired duty cycle
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
    
        DUTY_CYCLE_CALCULATION(timer); // Calculate CCR value for 50% duty cycle

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
        DUTY_CYCLE_CALCULATION(timer); // Calculate CCR value for 50% duty cycle
    }

    return HAL_OK;
}


//Motor structure 

typedef enum MOTOR_PIN{
    DIRECTION,
    STEP,
    ENABLE
};


struct Motor_Control{
    MOTOR_PIN motor_pin;
    uint8_t (*motor_direction_setter)(uint8_t motor_new_state);
    uint8_t (*motor_direction_getter)(void);
};

struct Motor_Control motor_control[3] = {
    [DIRECTION] = {
        .motor_pin = DIRECTION,
        .motor_direction_setter = motor_direction_setter,
        .motor_direction_getter = motor_direction_getter
    },
    [STEP] = {
        .motor_pin = STEP,
        .motor_direction_setter = motor_speed_setter, // No setter for step pin
        .motor_direction_getter = NULL  // No getter for step pin
    },
    [ENABLE] = {
        .motor_pin = ENABLE,
        .motor_direction_setter = motor_enable_setter,
        .motor_direction_getter = motor_enable_getter
    }
};


