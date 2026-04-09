#ifndef MOTOR_SETUP_FUNCTIONS_H
#define MOTOR_SETUP_FUNCTIONS_H

MOTOR_STATE motor_state_return(void);


extern struct Motor_Control motor_control[3];

typedef enum MOTOR_PIN{
    DIRECTION,
    STEP,
    ENABLE
};


#endif /* MOTOR_SETUP_FUNCTIONS_H */