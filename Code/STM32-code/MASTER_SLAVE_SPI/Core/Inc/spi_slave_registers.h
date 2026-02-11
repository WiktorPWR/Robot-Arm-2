/**
 * @file    spi_slave_registers.h
 * @brief   Register map and data storage for SPI Slave
 * @details Contains register data variables and function prototypes
 */

#ifndef SPI_SLAVE_REGISTERS_H
#define SPI_SLAVE_REGISTERS_H

#include <protocol/spi_slave_types.h>

/*******************************************************************************
 * REGISTER DATA STORAGE
 ******************************************************************************/

extern uint8_t reg_homing_data;           /**< Homing register: 0=idle, 1=start */
extern uint32_t reg_move_angle_data;      /**< Move angle register (4 bytes) */
extern uint8_t reg_diag_control_data;     /**< Diagnostics control register */
extern uint32_t reg_diag_status_data;     /**< Diagnostics status register */
extern uint8_t reg_emergency_stop_data;   /**< Emergency stop register */

/*******************************************************************************
 * REGISTER CHANGE FLAGS
 ******************************************************************************/

extern volatile Register_Change_Flags_t register_flags;

/*******************************************************************************
 * REGISTER ACCESS FUNCTION PROTOTYPES
 ******************************************************************************/

/* Register read functions */
void* homing_read(uint8_t *buf, uint8_t offset);
void* move_angle_read(uint8_t *buf, uint8_t offset);
void* diagnostics_status_read(uint8_t *buf, uint8_t offset);
void* emergency_stop_read(uint8_t *buf, uint8_t offset);

/* Register write functions */
void homing_write(uint8_t *buf, uint8_t offset, uint8_t value);
void move_angle_write(uint8_t *buf, uint8_t offset, uint8_t value);
void diagnostics_control_write(uint8_t *buf, uint8_t offset, uint8_t value);
void emergency_stop_write(uint8_t *buf, uint8_t offset, uint8_t value);

/* Register callbacks */
void homing_callback(void);
void move_angle_callback(void);
void diag_control_callback(void);
void emergency_stop_callback(void);

/*******************************************************************************
 * REGISTER MAP ACCESS
 ******************************************************************************/

extern const Register_Structure_t register_map[REG_COUNT];

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Monitor register changes and execute callbacks
 * @details Should be called continuously in main loop
 */
void monitor_register_changes(void);

#endif /* SPI_SLAVE_REGISTERS_H */
