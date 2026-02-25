/*
 * endstop.c
 *
 *  Created on: Feb 21, 2026
 *      Author: ostro
 */
#ifndef ENDSTOP_H
#define ENDSTOP_H

#include "main.h"

#define ENDSTOP_DEBOUNCE_MS  10

uint8_t endstop_is_pressed(void);


#endif
