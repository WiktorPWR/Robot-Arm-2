#include "motor_trajectory_modeling.h"
#include "motor_setup_functions.h"

typedef enum{
    NO_TRAJECTORY_SET,
    TRAPEZOIDAL,
    S_CURVE
}TRAJECTORY_TYPE;

typedef struct{
    TRAJECTORY_TYPE trajectory_type;
    float max_velocity;
    float max_acceleration;
    float max_jerk;
} Trajectory_Parameters;

static Trajectory_Parameters trajectory_params = {
    .trajectory_type = NO_TRAJECTORY_SET, // Default value, should be set according to the desired trajectory type
    .max_velocity = 0.0, // Example value, should be set according to the motor specifications
    .max_acceleration = 0.0, // Example value, should be set according to the motor specifications
    .max_jerk = 0.0 // Example value, should be set according to the motor specifications
};

float max_velocity_setter(float new_max_velocity){
    trajectory_params.max_velocity = new_max_velocity;
    if(trajectory_params.max_velocity < 0){
        trajectory_params.max_velocity = 0;
    }
    return trajectory_params.max_velocity;
}

float max_velocity_getter(void){
    return trajectory_params.max_velocity;
}

float max_acceleration_setter(float new_max_acceleration){
    trajectory_params.max_acceleration = new_max_acceleration;
    if(trajectory_params.max_acceleration < 0){
        trajectory_params.max_acceleration = 0;
    }
    return trajectory_params.max_acceleration;
}

float max_acceleration_getter(void){
    return trajectory_params.max_acceleration;
}

float max_jerk_setter(float new_max_jerk){
    trajectory_params.max_jerk = new_max_jerk;
    if(trajectory_params.max_jerk < 0){
        trajectory_params.max_jerk = 0;
    }
    return trajectory_params.max_jerk;
}

float max_jerk_getter(void){
    return trajectory_params.max_jerk;
}

TRAJECTORY_TYPE trajectory_type_setter(TRAJECTORY_TYPE new_trajectory_type){
    trajectory_params.trajectory_type = new_trajectory_type;
    return trajectory_params.trajectory_type;
}

TRAJECTORY_TYPE trajectory_type_getter(void){
    return trajectory_params.trajectory_type;
}

typedef struct{
    float (*getter)(void);
    float (*setter)(float new_value);
}Parameter;

typedef struct{
    TRAJECTORY_TYPE (*getter)(void);
    TRAJECTORY_TYPE (*setter)(TRAJECTORY_TYPE new_trajectory_type);
}Trajectory_Type_Control;

typedef struct{
    Trajectory_Type_Control trajectory_type_control;
    Parameter max_velocity_control;
    Parameter max_acceleration_control;
    Parameter max_jerk_control;
}Trajectory_Parameter_Control;

const Trajectory_Parameter_Control trajectory_parameter_control = {
    .trajectory_type_control = {
        .getter = trajectory_type_getter,
        .setter = trajectory_type_setter
    },
    .max_velocity_control = {
        .getter = max_velocity_getter,
        .setter = max_velocity_setter
    },
    .max_acceleration_control = {
        .getter = max_acceleration_getter,
        .setter = max_acceleration_setter
    },
    .max_jerk_control = {
        .getter = max_jerk_getter,
        .setter = max_jerk_setter
    }
};


//What was done:
// We have API for user to set trajectory max parateres. It is good becous user cant just chage max value 
// he have to use API for it. This architektrue have also one more advantege - we still can use object to use in movement fnction
// for this file. That is good becous it is much faster than using getter and setter for each parameter in movement function.


typedef struct{
    float current_position;
    float current_velocity;
    float current_acceleration;
    float current_jerk;
} Motor_Movement_Variables;

static Motor_Movement_Variables motor_movement_variables = {
    .current_position = 0.0,
    .current_velocity = 0.0,
    .current_acceleration = 0.0,
    .current_jerk = 0.0
};


//Here we add new possible movemet functions, we can add as many as we want, but for now we will implement only trapezoidal and s-curve trajectory

static void move_trapezoidal(float target_position);

static void move_s_curve(float target_position);

static void move_via_angle(float target_position){
    switch(trajectory_params.trajectory_type){
        case TRAPEZOIDAL:
            move_trapezoidal(target_position);
            break;
        case S_CURVE:
            move_s_curve(target_position);
            break;
        default:
            motor_state_setter(MOTOR_ERROR);
            // Handle error case, maybe set motor state to error
            break;
    }
}

typedef struct{
    const Trajectory_Parameter_Control *trajectory_parameter_control;
    Motor_Movement_Variables *motor_movement_variables;
    void (*move_via_angle)(float target_position);
}MOTOR;

MOTOR motor = {
    .trajectory_parameter_control = &trajectory_parameter_control,
    .motor_movement_variables = &motor_movement_variables,
    .move_via_angle = move_via_angle
}



// TO DO:
// - Implement move_trapezoidal function
// - Implement move_s_curve function


static void move_trapezoidal(float target_position){
    // Implement trapezoidal trajectory movement logic here
    // This function should update motor_movement_variables based on the target_position and trajectory parameters
}

static void move_s_curve(float target_position){
    // Implement s-curve trajectory movement logic here
    // This function should update motor_movement_variables based on the target_position and trajectory parameters
}










