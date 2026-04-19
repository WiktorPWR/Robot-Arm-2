#include "motor_setup_functions.h"


static const uint16_t DUTY_CYCLE = 50;
static const uint16_t START_FREQUENCY = 1000; // 1 kHz

extern TIM_HandleTypeDef htim3; // We need this to set the speed of the motor, we can change it to other timer if we want to use different timer for motor control

//This is functionality for state machine of the motor

static MOTOR_STATE_FLAGS motor_state = MOTOR_STOP;

MOTOR_STATE_FLAGS motor_state_getter(void){
    return motor_state;
};

static MOTOR_STATE_FLAGS motor_state_setter(MOTOR_STATE_FLAGS new_motor_state){
    motor_state = new_motor_state;
    return motor_state;
}


//This part is for setting direction of the motor 

MOTOR_DIRECTION_FLAGS motor_direction = NONE;

MOTOR_DIRECTION_FLAGS motor_direction_getter(void){
    return motor_direction;
};

MOTOR_DIRECTION_FLAGS motor_direction_setter(MOTOR_DIRECTION_FLAGS new_motor_direction){
    motor_direction = new_motor_direction;
    return motor_direction;
}

//This part is for enable motor or not


MOTOR_ENABLE_FLAGS motor_enable = DISABLED;

MOTOR_ENABLE_FLAGS motor_enable_getter(void){
    return motor_enable;
};

MOTOR_ENABLE_FLAGS motor_enable_setter(MOTOR_ENABLE_FLAGS new_motor_enable){
    motor_enable = new_motor_enable;
    return motor_enable;
}

//This function always set out value to 50% duty cycle, so we can change the frequency without changing the duty cycle
static inline void DUTY_CYCLE_CALCULATION(TIM_HandleTypeDef *timer){
    timer->Instance->CCR1 = (timer->Instance->ARR * DUTY_CYCLE / 100); // Calculate CCR value for desired duty cycle
}

//This part is for setting speed of the motor
HAL_StatusTypeDef motor_speed_setter(uint16_t frequency){

    // If frequency is 0, stop the motor
    if(frequency == 0){
        //We stop the timer to stop the motor
        if(HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1) != HAL_OK){
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
        htim3.Instance->ARR = 0;

        //Then we calculate the prescaler value based on the desired frequency and the timer clock frequency
        uint32_t APB1_CLK = HAL_RCC_GetPCLK1Freq();
        htim3.Instance->PSC = ((APB1_CLK / START_FREQUENCY) - 1); // Calculate prescaler for 1 kHz frequency
    
        DUTY_CYCLE_CALCULATION(&htim3); // Calculate CCR value for 50% duty cycle

        //When we have this set we can start PWM signal generation
        if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK){
            motor_state_setter(MOTOR_ERROR);
            return HAL_ERROR;
        }
        
        // Set the motor state to running
        motor_state_setter(MOTOR_RUNNING);

    }

    // if motor is running then we just update the arr value to change the frequency
    if(motor_state_getter() == MOTOR_RUNNING){
        uint32_t APB1_CLK = HAL_RCC_GetPCLK1Freq();
        htim3.Instance->ARR = (APB1_CLK / (frequency * (htim3.Instance->PSC + 1))) - 1; // Calculate ARR value for desired frequency
        DUTY_CYCLE_CALCULATION(&htim3); // Calculate CCR value for 50% duty cycle
    }

    return HAL_OK;
}


//Motor structure 
MOTOR motor = {
    .enable_control = {
        .pin = ENABLE_PIN,
        .motor_enable_getter = motor_enable_getter,
        .motor_enable_setter = motor_enable_setter
    },
    .direction_control = {
        .pin = DIRECTION_PIN,
        .motor_direction_getter = motor_direction_getter,
        .motor_direction_setter = motor_direction_setter
    },
    .step_control = {
        .pin = STEP_PIN,
        .motor_speed_setter = motor_speed_setter
    }
};