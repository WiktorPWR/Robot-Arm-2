/*
 * endstop.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */
#include "hardware/endstop.h"

uint8_t endstop_is_pressed(void)
{
    if (HAL_GPIO_ReadPin(ENDSTOP_GPIO_Port, ENDSTOP_Pin))
    {
        HAL_Delay(ENDSTOP_DEBOUNCE_MS);

        /* Sprawdzenie ponownie po czasie */
        if (HAL_GPIO_ReadPin(ENDSTOP_GPIO_Port, ENDSTOP_Pin))
        {
            return 1;   // stabilne wciśnięcie
        }
    }

    return 0;
}

