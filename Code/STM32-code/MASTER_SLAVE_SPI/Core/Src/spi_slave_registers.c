/**
 * @file    spi_slave_registers.c
 * @brief   Register implementation for SPI Slave
 * @details Contains register data, access functions, and callbacks
 */

#include "spi_slave_registers.h"
#include "main.h"
#include <string.h>

/*******************************************************************************
 * REGISTER DATA STORAGE (DEFINITIONS)
 ******************************************************************************/

uint8_t reg_homing_data = 0;                    /**< Homing register: 0=idle, 1=start */
uint32_t reg_move_angle_data = 0;               /**< Move angle register (4 bytes) */
uint8_t reg_diag_control_data = 0;              /**< Diagnostics control register */
uint32_t reg_diag_status_data = 0;              /**< Diagnostics status register */
uint8_t reg_emergency_stop_data = 0;            /**< Emergency stop register */

/*******************************************************************************
 * REGISTER CHANGE FLAGS (DEFINITION)
 ******************************************************************************/

volatile Register_Change_Flags_t register_flags = {0};

/*******************************************************************************
 * REGISTER IMPLEMENTATION - HOMING
 ******************************************************************************/

void* homing_read(uint8_t *buf, uint8_t offset)
{
    if (buf != NULL && offset == 0) {
        buf[0] = reg_homing_data;
    }
    return (void*)&reg_homing_data;
}

void homing_write(uint8_t *buf, uint8_t offset, uint8_t value)
{
    if (offset == 0) {
        reg_homing_data = value;
        register_flags.homing_changed = 1;
    }
}

void homing_callback(void)
{
    /* TODO: Implement homing operation */
    
    if (reg_homing_data == 1) {
        // Przykład implementacji:
        // 1. Rozpocznij procedurę homing
        // 2. Ruch do pozycji home
        // 3. Czekaj na sensor home
        // 4. Zatrzymaj silnik
        // 5. Wyzeruj pozycję
        
        // Po zakończeniu:
        reg_homing_data = 0;
    }
    
    /* Set GPIO to indicate ready for new frame */
    HAL_GPIO_WritePin(READY_SLAVE_FLAG_GPIO_Port, 
                      READY_SLAVE_FLAG_Pin, 
                      GPIO_PIN_RESET);
}

/*******************************************************************************
 * REGISTER IMPLEMENTATION - MOVE ANGLE
 ******************************************************************************/

void* move_angle_read(uint8_t *buf, uint8_t offset)
{
    if (buf != NULL && offset < 4) {
        buf[0] = (reg_move_angle_data >> (8 * (3 - offset))) & 0xFF;
    }
    return (void*)&reg_move_angle_data;
}

void move_angle_write(uint8_t *buf, uint8_t offset, uint8_t value)
{
    if (offset < 4) {
        /* Clear the byte at offset and set new value */
        uint32_t mask = ~(0xFF << (8 * (3 - offset)));
        reg_move_angle_data = (reg_move_angle_data & mask) | (value << (8 * (3 - offset)));
        
        /* Set flag when last byte is written */
        if (offset == 3) {
            register_flags.move_angle_changed = 1;
        }
    }
}

void move_angle_callback(void)
{
    /* TODO: Implement angle movement operation */
    
    // Przykład implementacji:
    // 1. Pobierz docelowy kąt
    // int32_t target_angle = (int32_t)reg_move_angle_data;
    
    // 2. Oblicz różnicę do aktualnej pozycji
    // int32_t angle_diff = target_angle - current_position;
    
    // 3. Ustaw kierunek ruchu
    // 4. Wykonaj ruch
    // 5. Aktualizuj aktualną pozycję
    
    /* Set GPIO to indicate ready for new frame */
    HAL_GPIO_WritePin(READY_SLAVE_FLAG_GPIO_Port, 
                      READY_SLAVE_FLAG_Pin, 
                      GPIO_PIN_RESET);
}

/*******************************************************************************
 * REGISTER IMPLEMENTATION - DIAGNOSTICS CONTROL
 ******************************************************************************/

void diagnostics_control_write(uint8_t *buf, uint8_t offset, uint8_t value)
{
    if (offset == 0) {
        reg_diag_control_data = value;
        register_flags.diag_control_changed = 1;
    }
}

void diag_control_callback(void)
{
    /* TODO: Implement diagnostics operation */
    
    // Przykład implementacji:
    // 1. Wykonaj testy diagnostyczne
    // 2. Zapisz wyniki do reg_diag_status_data
    // 3. Ustaw odpowiednie bity statusu
    
    // Przykładowe bity diagnostyczne:
    // Bit 0: Motor OK
    // Bit 1: Sensor OK
    // Bit 2: Communication OK
    // Bit 3: Position OK
    
    /* Set GPIO to indicate ready for new frame */
    HAL_GPIO_WritePin(READY_SLAVE_FLAG_GPIO_Port, 
                      READY_SLAVE_FLAG_Pin, 
                      GPIO_PIN_RESET);
}

/*******************************************************************************
 * REGISTER IMPLEMENTATION - DIAGNOSTICS STATUS
 ******************************************************************************/

void* diagnostics_status_read(uint8_t *buf, uint8_t offset)
{
    if (buf != NULL && offset < 4) {
        buf[0] = (reg_diag_status_data >> (8 * (3 - offset))) & 0xFF;
    }
    return (void*)&reg_diag_status_data;
}

/*******************************************************************************
 * REGISTER IMPLEMENTATION - EMERGENCY STOP
 ******************************************************************************/

void* emergency_stop_read(uint8_t *buf, uint8_t offset)
{
    if (buf != NULL && offset == 0) {
        buf[0] = reg_emergency_stop_data;
    }
    return (void*)&reg_emergency_stop_data;
}

void emergency_stop_write(uint8_t *buf, uint8_t offset, uint8_t value)
{
    if (offset == 0) {
        reg_emergency_stop_data = value;
        register_flags.emergency_stop_changed = 1;
    }
}

void emergency_stop_callback(void)
{
    /* CRITICAL: Implement emergency stop operation */
    
    if (reg_emergency_stop_data == 1) {
        // EMERGENCY STOP ACTIVATED
        // 1. Natychmiastowe zatrzymanie wszystkich silników
        // 2. Wyłączenie wszystkich PWM
        // 3. Ustawienie flagi bezpieczeństwa
        // 4. LED ostrzegawczy ON
    } else {
        // EMERGENCY STOP DEACTIVATED
        // 1. Reset flagi bezpieczeństwa
        // 2. LED ostrzegawczy OFF
        // 3. System gotowy do pracy
    }
    
    /* Set GPIO to indicate ready for new frame */
    HAL_GPIO_WritePin(READY_SLAVE_FLAG_GPIO_Port, 
                      READY_SLAVE_FLAG_Pin, 
                      GPIO_PIN_RESET);
}

/*******************************************************************************
 * REGISTER MAP DEFINITION
 ******************************************************************************/

const Register_Structure_t register_map[REG_COUNT] = {
    [REG_HOMING] = {
        .read_function = homing_read,
        .write_function = homing_write,
        .callback = homing_callback,
        .size = 1,
        .flags = REGISTER_READ_WRITE
    },
    [REG_MOVE_ANGLE] = {
        .read_function = move_angle_read,
        .write_function = move_angle_write,
        .callback = move_angle_callback,
        .size = 4,
        .flags = REGISTER_READ_WRITE
    },
    [REG_DIAG_CONTROL] = {
        .read_function = NULL,
        .write_function = diagnostics_control_write,
        .callback = diag_control_callback,
        .size = 1,
        .flags = REGISTER_WRITE
    },
    [REG_DIAG_STATUS] = {
        .read_function = diagnostics_status_read,
        .write_function = NULL,
        .callback = NULL,
        .size = 4,
        .flags = REGISTER_READ
    },
    [REG_EMERGENCY_STOP] = {
        .read_function = emergency_stop_read,
        .write_function = emergency_stop_write,
        .callback = emergency_stop_callback,
        .size = 1,
        .flags = REGISTER_READ_WRITE
    }
};

/*******************************************************************************
 * PUBLIC FUNCTIONS - REGISTER MONITORING
 ******************************************************************************/

/**
 * @brief Monitor register changes and execute callbacks
 * @details Should be called continuously in main loop
 */
void monitor_register_changes(void)
{
    /* Check homing register */
    if (register_flags.homing_changed) {
        register_flags.homing_changed = 0;
        if (register_map[REG_HOMING].callback != NULL) {
            register_map[REG_HOMING].callback();
        }
    }

    /* Check move angle register */
    if (register_flags.move_angle_changed) {
        register_flags.move_angle_changed = 0;
        if (register_map[REG_MOVE_ANGLE].callback != NULL) {
            register_map[REG_MOVE_ANGLE].callback();
        }
    }

    /* Check diagnostics control register */
    if (register_flags.diag_control_changed) {
        register_flags.diag_control_changed = 0;
        if (register_map[REG_DIAG_CONTROL].callback != NULL) {
            register_map[REG_DIAG_CONTROL].callback();
        }
    }

    /* Check emergency stop register */
    if (register_flags.emergency_stop_changed) {
        register_flags.emergency_stop_changed = 0;
        if (register_map[REG_EMERGENCY_STOP].callback != NULL) {
            register_map[REG_EMERGENCY_STOP].callback();
        }
    }
}
