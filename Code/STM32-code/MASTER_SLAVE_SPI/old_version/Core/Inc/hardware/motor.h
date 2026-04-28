/*
 * motor.h
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* =========================================================
 * External timer handles (defined in main.c / tim.c)
 * ========================================================= */

/* PWM timer — generates the STEP signal for the stepper motor driver */
extern TIM_HandleTypeDef htim2; /* FIX: was "htmi2" (typo) */

/* PID regulator timer — triggers an interrupt every 1/HERTZ_REGULATOR seconds */
extern TIM_HandleTypeDef htim4;

/* =========================================================
 * Motor and microstepping parameters
 * ========================================================= */

#define MICROSTEPPING_VALUE 16       /* Driver microstepping setting */
#define MAX_SPEED           100      /* Maximum motor speed [Hz] */
#define MAX_ACCEL           10       /* Maximum acceleration */
#define MAX_DECEL           10       /* Maximum deceleration */
#define MAX_JERK            10       /* Maximum jerk (rate of acceleration change) */

/* =========================================================
 * Timer parameters
 * ========================================================= */

#define TIM_CLK_VALUE   16000000UL  /* Timer input clock frequency [Hz] — 16 MHz */
#define TIMER_CHANNEL   TIM_CHANNEL_1

/*
 * Prescaler value for the PID regulator timer.
 * The PSC register requires a value one less than the desired division factor,
 * hence the -1.
 * Example: prescaler = 500 → PSC = 499
 */
#define PRESCALER_VALUE (500U - 1U)

/* Position tolerance [encoder steps] — accepted range is ±POSITION_TOLERANCES */
#define POSITION_TOLERANCES 2U

/* =========================================================
 * PID regulator parameters
 * ========================================================= */

/*
 * PID regulator call frequency [Hz].
 * Timer htim4 fires an interrupt at this rate;
 * the ISR sets the new_cycle flag to trigger PID computation.
 */
#define HERTZ_REGULATOR 1000U /* FIX: removed duplicate definition */

/* PID gains (floating-point values, scaled before use) */
#define KP   2.0f
#define KI   0.1f
#define KD   0.01f

/* Integrator clamp limits (anti-windup) */
#define I_MAX  10
#define I_MIN  0

/*
 * PID fixed-point scale factor.
 * Because we use integer arithmetic (int32_t) instead of float inside
 * the control loop, all coefficients are multiplied by PID_SCALE to
 * preserve sufficient precision.
 */
#define PID_SCALE 1000

/* =========================================================
 * New-cycle flag (set by htim4 ISR, cleared by the control loop)
 * ========================================================= */
extern volatile uint8_t new_cycle;

/* =========================================================
 * Types
 * ========================================================= */

/* Motor rotation direction */
typedef enum {
    LEFT  = 67, /* Counter-clockwise — maps to the corresponding GPIO state */
    RIGHT = 69  /* Clockwise         — maps to the corresponding GPIO state */
} Rotation_Direction;

/*
 * PID regulator structure (fixed-point arithmetic).
 * All coefficients are pre-multiplied by the 'scale' field so that
 * the control loop can operate entirely on integers without floats.
 */
typedef struct {
    int32_t Kp;          /* Proportional gain (scaled) */
    int32_t Ki;          /* Integral gain     (scaled) */
    int32_t Kd;          /* Derivative gain   (scaled) */
    int32_t dt;          /* Time step         (scaled) */

    int32_t scale;       /* Common scale factor (e.g. 1000) */

    int32_t prev_value;  /* Previous error — required for the D term */
    int32_t error_value; /* Current error: setpoint - measured value */

    int32_t P;           /* Proportional term output */
    int32_t I;           /* Integral term output */
    int32_t D;           /* Derivative term output */

    int32_t integral;    /* Integrator accumulator (before scaling) */
    int32_t derivative;  /* Error derivative */

    /* Anti-windup: integrator output limits */
    int32_t I_max;
    int32_t I_min;

    int64_t output;      /* Regulator output — STEP signal frequency [Hz] */
} PID_Regulator_t;

/* =========================================================
 * Global PID regulator instance.
 * FIX: declared as extern here; defined once in motor.c to avoid
 *      "multiple definition" linker errors on repeated #include.
 * ========================================================= */
extern PID_Regulator_t pid_regulator;

/* =========================================================
 * Function prototypes
 * ========================================================= */

/**
 * @brief  Initialises the PID regulator structure with default values.
 * @param  reg  Pointer to the PID_Regulator_t instance to initialise.
 */
void PID_init(PID_Regulator_t *reg);

/**
 * @brief  Sets the stepper motor speed by adjusting the PWM frequency.
 * @param  freq  Desired STEP signal frequency [Hz]. 0 stops the motor.
 */
void stepper_set_speed(int64_t freq);

/**
 * @brief  Rotates the motor to the target position using PID control.
 * @param  angle_value        Target position [encoder steps].
 * @param  rotation_direction Direction of rotation (LEFT or RIGHT).
 */
void move_via_angle(float angle_value, Rotation_Direction rotation_direction);

/**
 * @brief  Rotates the motor at minimum speed in the given direction.
 * @param  rotation_direction Direction of rotation (LEFT or RIGHT).
 */
void move_minimal_speed(Rotation_Direction rotation_direction);


void homing(void);

#endif /* MOTOR_H */
