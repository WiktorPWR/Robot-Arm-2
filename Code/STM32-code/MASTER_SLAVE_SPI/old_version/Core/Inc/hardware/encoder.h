/*
 * encoder.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

extern TIM_HandleTypeDef htim3;

#define ENCODER_STEPS_PER_REV  600U    // 600 PPR, bez quadrature
#define GEAR_RATIO             2U
#define ANGLE_SCALE            1000U   // rozdzielczość 0.001°
#define POSITION_TOLERANCES_ENCODER    5       // ±0.005° tolerancja


int32_t get_current_value(void);

#endif
