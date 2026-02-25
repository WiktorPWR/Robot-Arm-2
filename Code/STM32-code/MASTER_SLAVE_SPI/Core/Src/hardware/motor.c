/*
 * motor.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */

#include "main.h"
#include "hardware/motor.h"
#include "hardware/encoder.h"
#include "hardware/endstop.h"
#include "spi_slave_registers.h"
#include "protocol/spi_slave_protocol.h"

/* =========================================================
 * Global PID regulator instance.
 * Declared as extern in motor.h.
 * ========================================================= */
PID_Regulator_t pid_regulator;

/*
 * New-cycle flag.
 * Set to 1 inside the htim4 ISR every 1/HERTZ_REGULATOR seconds.
 * Cleared by the control loop after each PID computation.
 *
 * Declared volatile so the compiler does not cache the value in a
 * register and misses updates written by the interrupt handler.
 */
volatile uint8_t new_cycle = 0;

/* =========================================================
 * Local error codes for the timer configuration helper
 * ========================================================= */
typedef enum {
    TIMER_OK,               /* Operation succeeded */
    TIMER_ERR_WRONG_HERTZ   /* Requested frequency is invalid (0) */
} Timer_Setting_Error;

/* =========================================================
 * stepper_set_speed
 * ========================================================= */

/**
 * @brief  Sets the PWM frequency of the STEP signal for the motor driver.
 *
 * Calculates the ARR and CCR register values for htim2 based on the
 * requested frequency. The PWM duty cycle is fixed at 50 %.
 *
 * @param  freq  Desired frequency [Hz].
 *               Passing 0 disables the STEP pulses (CCR = 0) without
 *               stopping the timer itself.
 */
void stepper_set_speed(int64_t freq)
{
    if (freq == 0)
    {
        /* Setting CCR to 0 disables the PWM output — no STEP pulses are sent */
        __HAL_TIM_SET_COMPARE(&htim2, TIMER_CHANNEL, 0);
        return;
    }

    /*
     * ARR (Auto-Reload Register) calculation:
     *   ARR = (f_timer / f_step) - 1
     * The -1 accounts for the timer counting from 0 to ARR inclusive.
     */
    uint32_t arr = (TIM_CLK_VALUE / (uint32_t)freq) - 1U;

    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);

    /* 50 % duty cycle — CCR is set to half the period */
    __HAL_TIM_SET_COMPARE(&htim2, TIMER_CHANNEL, arr / 2U);
}

void homing(void)
{
    stepper_set_speed(5);

    while (!endstop_is_pressed())
    {
        /* czekamy aż krańcówka zostanie stabilnie wciśnięta */
    }

    stepper_set_speed(0);
}


/* =========================================================
 * setting_regulator_timer  (file-local helper)
 * ========================================================= */

/**
 * @brief  Configures htim4 as the PID regulator tick source.
 *
 * Sets the prescaler and ARR so that the timer overflow interrupt fires
 * at the requested frequency. The timer is (re)started after configuration.
 *
 * @param  hertz  Desired interrupt frequency [Hz].
 * @return TIMER_OK on success, TIMER_ERR_WRONG_HERTZ if hertz == 0.
 */
static Timer_Setting_Error setting_regulator_timer(uint16_t hertz) /* FIX: removed stray leading comma */
{
    if (hertz == 0)
    {
        return TIMER_ERR_WRONG_HERTZ;
    }

    /* Stop the timer before changing its registers to avoid race conditions */
    HAL_TIM_Base_Stop_IT(&htim4);

    /*
     * Apply the prescaler.
     * PRESCALER_VALUE = 499  →  effective clock = 16 MHz / 500 = 32 kHz.
     * FIX: added & before htim4 — the macro requires a pointer.
     */
    __HAL_TIM_SET_PRESCALER(&htim4, PRESCALER_VALUE);

    /*
     * ARR calculation:
     *   f_after_prescaler = TIM_CLK_VALUE / (PRESCALER_VALUE + 1)
     *   ARR = f_after_prescaler / hertz
     *
     * Example — TIM_CLK_VALUE = 16 MHz, PRESCALER_VALUE = 499, hertz = 1000:
     *   f_after_prescaler = 32 000 Hz  →  ARR = 32
     *
     * NOTE: if the result exceeds 65535 (uint16_t limit), the value will
     * wrap — choose PRESCALER_VALUE accordingly.
     */
    uint16_t arr_value = (uint16_t)((TIM_CLK_VALUE / (PRESCALER_VALUE + 1U)) / hertz);

    /* FIX: added & before htim4 */
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr_value);

    /* Reset the counter so the first cycle starts cleanly from 0 */
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    /* FIX: timer was never restarted in the original code */
    HAL_TIM_Base_Start_IT(&htim4);

    return TIMER_OK;
}

/* =========================================================
 * PID_init
 * ========================================================= */

/**
 * @brief  Initialises the PID regulator with default coefficients.
 *
 * All gains are multiplied by reg->scale so that subsequent calculations
 * inside the control loop can use integer arithmetic exclusively.
 *
 * @param  reg  Pointer to the PID_Regulator_t to initialise.
 *              FIX: parameter changed from pass-by-value to pointer —
 *                   the original code modified a local copy only.
 */
void PID_init(PID_Regulator_t *reg)
{
    reg->scale = PID_SCALE;

    /* Convert floating-point gains to fixed-point representation */
    reg->Kp = (int32_t)(KP * (float)reg->scale);
    reg->Ki = (int32_t)(KI * (float)reg->scale);
    reg->Kd = (int32_t)(KD * (float)reg->scale);

    /* dt = 1/f [s], stored in fixed-point for completeness */
    reg->dt = (int32_t)(1.0f / HERTZ_REGULATOR * (float)reg->scale);

    /* Integrator clamp limits (anti-windup) */
    reg->I_max = (int32_t)(I_MAX * reg->scale);
    reg->I_min = (int32_t)(I_MIN * reg->scale);

    /* Clear all state variables */
    reg->error_value = 0;
    reg->prev_value  = 0;
    reg->P           = 0;
    reg->I           = 0;
    reg->D           = 0;
    reg->integral    = 0;
    reg->derivative  = 0;
    reg->output      = 0;
}

/* =========================================================
 * move_via_angle
 * ========================================================= */

/**
 * @brief  Drives the motor to the target position with PID position control.
 *
 * Blocking function — returns only when the encoder reading falls within
 * [angle_value - POSITION_TOLERANCES, angle_value + POSITION_TOLERANCES].
 *
 * The PID regulator is executed synchronously with the new_cycle flag,
 * which is set by the htim4 ISR at HERTZ_REGULATOR Hz.
 *
 * @param  angle_value        Target position [encoder steps].
 * @param  rotation_direction Direction of rotation (LEFT or RIGHT).
 */
void move_via_angle(float angle_value, Rotation_Direction rotation_direction)
{
    // Przelicz kąt float -> int32 ze skalowaniem
    // np. 90.5° -> 90500 jednostek
    int32_t target_scaled = (int32_t)(angle_value * (float)ANGLE_SCALE);

    HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin,
                      (rotation_direction == RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (register_map[REG_ENABLE_MOTOR].read_function(NULL, 0) == 0)
        return;

    PID_init(&pid_regulator);

    if (setting_regulator_timer(HERTZ_REGULATOR) != TIMER_OK)
        return;

    while (1)
    {
        // get_current_value() zwraca już przeskalowany kąt w jednostkach 0.001°
        int32_t current_scaled = get_current_value();

        if (current_scaled >= (target_scaled - POSITION_TOLERANCES) &&
            current_scaled <= (target_scaled + POSITION_TOLERANCES))
            break;

        if (new_cycle == 1)
        {
            current_scaled = get_current_value();

            // Error w jednostkach 0.001°
            pid_regulator.error_value = target_scaled - current_scaled;

            // P
            pid_regulator.P = pid_regulator.Kp * pid_regulator.error_value;

            // I z anti-windup
            if (pid_regulator.integral + pid_regulator.error_value > pid_regulator.I_max)
                pid_regulator.integral = pid_regulator.I_max;
            else if (pid_regulator.integral + pid_regulator.error_value < pid_regulator.I_min)
                pid_regulator.integral = pid_regulator.I_min;
            else
                pid_regulator.integral += pid_regulator.error_value;

            pid_regulator.I = pid_regulator.Ki * pid_regulator.integral;

            // D
            pid_regulator.derivative = pid_regulator.error_value - pid_regulator.prev_value;
            pid_regulator.D = pid_regulator.Kd * pid_regulator.derivative;

            // Output — int64 żeby nie przepełnić
            pid_regulator.output = (int64_t)pid_regulator.P
                                 + (int64_t)pid_regulator.I
                                 + (int64_t)pid_regulator.D;

            // Dzielimy tylko przez PID_SCALE
            // ANGLE_SCALE jest już wbudowane w get_current_value()
            pid_regulator.output /= pid_regulator.scale;

            if (pid_regulator.output < 0)         pid_regulator.output = 0;
            if (pid_regulator.output > MAX_SPEED)  pid_regulator.output = MAX_SPEED;

            stepper_set_speed(pid_regulator.output);

            pid_regulator.prev_value  = pid_regulator.error_value;
            pid_regulator.error_value = 0;

            new_cycle = 0;
            timer_reset(&htim4);
        }
    }

    stepper_set_speed(0);
    HAL_TIM_Base_Stop_IT(&htim4);
}
