/**
 * @file    spi_slave_types.h
 * @brief   Type definitions and constants for SPI Slave communication
 * @details Contains all enumerations, structures, and constant definitions
 */

#ifndef SPI_SLAVE_TYPES_H
#define SPI_SLAVE_TYPES_H

#include <stdint.h>

/*******************************************************************************
 * PROTOCOL CONSTANTS
 ******************************************************************************/

/** @defgroup Protocol_Constants Protocol byte constants */
/** @{ */
#define START_BYTE          0x55    /**< Frame start delimiter */
#define SLAVE_ID            0x01    /**< Slave device identifier */
#define END_BYTE            0xAA    /**< Frame end delimiter */
#define SYN_BYTE            0x97    /**< Synchronization request byte */
#define SYN_ACK_BYTE        0x98    /**< Synchronization acknowledgment byte */
#define ACK_BYTE            0x99    /**< Acknowledgment byte */
#define COMMIT_BYTE         0x9A    /**< Commit confirmation byte */
/** @} */

/** @defgroup Buffer_Sizes Buffer size definitions */
/** @{ */
#define HANDSHAKE_SIZE              1
#define DATA_SIZE                   16      /**< Maximum data payload size */
#define COMMIT_FRAME_SIZE           4       /**< START_BYTE + commit + error_code + END_BYTE */
#define SIZE_OF_CRC                 4       /**< CRC-32 size in bytes */
/** @} */

/** @defgroup Operation_Types Operation type definitions */
/** @{ */
#define OPERATION_READ      0x01    /**< Read operation from slave */
#define OPERATION_WRITE     0x02    /**< Write operation to slave */
/** @} */

/*******************************************************************************
 * HEADER STRUCTURE
 ******************************************************************************/

/**
 * @brief Communication frame header structure
 */
typedef struct {
    uint8_t start_byte;             /**< Frame start byte (0x55) */
    uint8_t slave_id;               /**< Target slave ID */
    uint8_t frame_id[2];            /**< Frame sequence number (16-bit) */
    uint8_t operation_type;         /**< Read (0x01) or write (0x02) operation */
    uint8_t register_address[2];    /**< Register address (16-bit) */
    uint8_t data_size[2];           /**< Number of bytes to read/write (16-bit) */
    uint8_t crc[SIZE_OF_CRC];       /**< CRC-32 of header fields */
    uint8_t end_byte;               /**< Frame end byte (0xAA) */
} Header_t;

#define HEADER_SIZE (sizeof(Header_t))
#define HEADER_SIZE_WITHOUT_CRC (HEADER_SIZE - SIZE_OF_CRC - 2)

/*******************************************************************************
 * ENUMERATIONS
 ******************************************************************************/

/**
 * @brief SPI communication process stages
 */
typedef enum {
    /* Handshake phase */
    WAIT_SYN,                   /**< Waiting for SYN byte from master */
    SEND_SYN_ACK,               /**< Sending SYN_ACK to master */
    WAIT_ACK,                   /**< Waiting for ACK from master */

    /* Header phase */
    WAIT_HEADER,                /**< Waiting for header from master */
    SEND_VALIDATION_CODE,       /**< Sending header validation result */

    /* Master writes to slave (slave receives data) */
    WAIT_DATA,                  /**< Waiting for data from master */
    SEND_COMMIT,                /**< Sending commit with error code */

    /* Master reads from slave (slave sends data) */
    SEND_DATA,                  /**< Sending data to master */
    WAIT_COMMIT                 /**< Waiting for commit confirmation */
} SPI_ProcessStage_t;

/**
 * @brief Frame validation error codes
 */
typedef enum {
    OK_FRAME = 0x00,            /**< Frame is valid */
    WRONG_START_BYTE = 0x01,    /**< Invalid start byte */
    WRONG_FRAME_ID = 0x02,      /**< Duplicate or invalid frame ID */
    WRONG_SLAVE_ID = 0x03,      /**< Wrong slave address */
    WRONG_REGISTER = 0x04,      /**< Unknown or inaccessible register */
    WRONG_DATA_SIZE = 0x05,     /**< Data size exceeds maximum or register size */
    WRONG_END_BYTE = 0x06,      /**< Invalid end byte */
    WRONG_CRC_VALUE = 0x07,     /**< CRC mismatch */
    WRONG_OPERATION = 0x08      /**< Invalid operation type for register */
} Frame_Errors_t;

/**
 * @brief Timeout timer state
 */
typedef enum {
    TIMER_START,                /**< Timer not started */
    TIMER_RUNNING,              /**< Timer is running */
    TIMER_STOP                  /**< Timer stopped */
} Timer_State_t;

/**
 * @brief Register identifiers
 */
typedef enum {
    REG_HOMING = 0,             /**< Homing operation control */
    REG_MOVE_ANGLE,             /**< Angle movement command */
    REG_DIAG_CONTROL,           /**< Diagnostics control (write-only) */
    REG_DIAG_STATUS,            /**< Diagnostics status (read-only) */
    REG_EMERGENCY_STOP,         /**< Emergency stop control */
    
    REG_COUNT                   /**< Total number of registers */
} Register_ID_t;

/**
 * @brief Register access permissions
 */
typedef enum {
    REGISTER_WRITE = 0x01,          /**< Write-only access */
    REGISTER_READ = 0x02,           /**< Read-only access */
    REGISTER_READ_WRITE = 0x03      /**< Read and write access */
} Register_Flag_t;

/*******************************************************************************
 * REGISTER STRUCTURES
 ******************************************************************************/

/**
 * @brief Register change flags for monitoring
 */
typedef struct {
    uint8_t homing_changed;         /**< Homing register changed flag */
    uint8_t move_angle_changed;     /**< Move angle register changed flag */
    uint8_t diag_control_changed;   /**< Diagnostics control changed flag */
    uint8_t emergency_stop_changed; /**< Emergency stop changed flag */
} Register_Change_Flags_t;

/* Function pointer types for register access */
typedef void* (*reg_read_fn)(uint8_t *buf, uint8_t offset);
typedef void (*reg_write_fn)(uint8_t *buf, uint8_t offset, uint8_t value);
typedef void (*reg_callback_fn)(void);

/**
 * @brief Register structure definition
 */
typedef struct {
    reg_read_fn read_function;      /**< Function to read register byte */
    reg_write_fn write_function;    /**< Function to write register byte */
    reg_callback_fn callback;       /**< Callback when register changes */
    uint8_t size;                   /**< Register size in bytes */
    uint8_t flags;                  /**< Access permissions */
} Register_Structure_t;

#endif /* SPI_SLAVE_TYPES_H */
