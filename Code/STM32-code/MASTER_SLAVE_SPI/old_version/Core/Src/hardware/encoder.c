/*
 * encoder.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */

#include "main.h"
#include "hardware/encoder.h"


int32_t get_current_value(void)
{
    uint32_t raw = __HAL_TIM_GET_COUNTER(&htim3);

    // raw -> kąt w jednostkach 0.001°
    // kąt = (raw / GEAR_RATIO / ENCODER_STEPS_PER_REV) * 360 * ANGLE_SCALE
    int32_t angle_scaled = (int32_t)(
        ((int64_t)raw * 360 * ANGLE_SCALE) /
        ((int64_t)GEAR_RATIO * ENCODER_STEPS_PER_REV)
    );

    return angle_scaled;
}
