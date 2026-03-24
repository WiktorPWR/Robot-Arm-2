#include "bootloader_functions.h"

void go_to_bootloader(void){
    // Deklarujemy wskaźnik na funkcję
    void (*BOOTLOADER_RESET_HANDLER)(void);

    // 1. Wyłącz przerwania
    __disable_irq();

    // 2. Zatrzymaj SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // 3. Resetuj konfigurację zegarów
    HAL_RCC_DeInit();

    // 4. Deinicjalizacja HAL
    HAL_DeInit();

    // 5. Wyczyść flagi przerwań w NVIC
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // 6. Ustaw Wskaźnik Stosu (MSP) - bierzesz go spod adresu 0x08000000
    __set_MSP(*(volatile uint32_t*)BOOTLOADER_START_ADDRESS);

    // 7. Pobierz adres Reset Handlera (adres 0x08000004)
    uint32_t jump_address = *(volatile uint32_t*)(BOOTLOADER_START_ADDRESS + 4);
    BOOTLOADER_RESET_HANDLER = (void (*)(void))jump_address;

    // 8. Skacz! - TUTAJ BYŁ BŁĄD. Musisz wywołać tę funkcję:
    BOOTLOADER_RESET_HANDLER(); 
    
}