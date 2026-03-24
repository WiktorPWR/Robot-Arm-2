#ifndef BOOTLOADER_FUNCTIONS_H
#define BOOTLOADER_FUNCTIONS_H

void go_to_bootloader(void);

void validate_application(void);

void jump_to_application(void);

HAL_StatusTypeDef flash_erase_application(void);

HAL_StatusTypeDef flash_write_application(uint32_t address, uint8_t* data, uint32_t length);

#endif