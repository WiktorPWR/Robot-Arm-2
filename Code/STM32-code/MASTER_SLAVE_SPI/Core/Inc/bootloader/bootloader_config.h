#ifndef BOOTLOADER_CONFIG_H
#define BOOTLOADER_CONFIG_H

#define BOOTLOADER_START_ADDRESS 0x08000000  /**< Start address of the bootloader in flash memory */
#define APPLICATION_HEADER_ADR   0x08004000  /**< Address of the application header */
#define HEADER_SIZE              0x1000      /**< Size of the application header (4 KB) */
#define APPLICATION_START_ADDRESS (APPLICATION_HEADER_ADR + HEADER_SIZE) /**< Start address of the application firmware */


#endif /* BOOTLOADER_CONFIG_H */