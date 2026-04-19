#ifndef MOTOR_SETUP_FUNCTIONS_H
#define MOTOR_SETUP_FUNCTIONS_H

#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"

typedef enum{
    DIRECTION_PIN,
    STEP_PIN,
    ENABLE_PIN
}MOTOR_PIN;

typedef enum{
    NOT_SET,
    DISABLED,
    ENABLED
}MOTOR_ENABLE_FLAGS;

typedef enum{
    NONE,
    LEFT,
    RIGHT
}MOTOR_DIRECTION_FLAGS;

typedef enum{
    MOTOR_STOP,
    MOTOR_RUNNING,
    MOTOR_ERROR
}MOTOR_STATE_FLAGS;

typedef struct{
    MOTOR_PIN pin;
    MOTOR_ENABLE_FLAGS (*motor_enable_getter)(void);
    MOTOR_ENABLE_FLAGS (*motor_enable_setter)(MOTOR_ENABLE_FLAGS new_motor_enable);
}Motor_ENABLE_Control;


typedef struct{
    MOTOR_PIN pin;
    MOTOR_DIRECTION_FLAGS (*motor_direction_getter)(void);
    MOTOR_DIRECTION_FLAGS (*motor_direction_setter)(MOTOR_DIRECTION_FLAGS new_motor_direction);
}Motor_DIRECTION_Control;


typedef struct{
    MOTOR_PIN pin;
    HAL_StatusTypeDef (*motor_speed_setter)(uint16_t frequency);
}Motor_STEP_Control;


typedef struct{
    Motor_ENABLE_Control enable_control;
    Motor_DIRECTION_Control direction_control;
    Motor_STEP_Control step_control;
}MOTOR;

extern MOTOR motor;

#endif /* MOTOR_SETUP_FUNCTIONS_H */