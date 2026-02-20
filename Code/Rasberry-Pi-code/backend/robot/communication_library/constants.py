"""
constants.py – Stałe protokołu SPI STM32
=========================================
Źródło: spi_slave_types.h
"""

# ---------------------------------------------------------------------------
# Bajty specjalne protokołu
# ---------------------------------------------------------------------------
START_BYTE   = 0x55   # Pierwszy bajt każdej ramki
END_BYTE     = 0xAA   # Ostatni bajt każdej ramki
SLAVE_ID     = 0x01   # Identyfikator slave'a (STM32)

SYN_BYTE     = 0x97   # Master → Slave: inicjacja handshake
SYN_ACK_BYTE = 0x98   # Slave → Master: potwierdzenie SYN
ACK_BYTE     = 0x99   # Master → Slave: zakończenie handshake
COMMIT_BYTE  = 0x9A   # Bajt ramki COMMIT (obie strony)

# ---------------------------------------------------------------------------
# Typy operacji (pole operation_type w headerze)
# ---------------------------------------------------------------------------
OPERATION_READ  = 0x01
OPERATION_WRITE = 0x02

# ---------------------------------------------------------------------------
# Kody błędów walidacji (Frame_Errors_t)
# ---------------------------------------------------------------------------
OK_FRAME          = 0x00
WRONG_START_BYTE  = 0x01
WRONG_FRAME_ID    = 0x02
WRONG_SLAVE_ID    = 0x03
WRONG_REGISTER    = 0x04
WRONG_DATA_SIZE   = 0x05
WRONG_END_BYTE    = 0x06
WRONG_CRC_VALUE   = 0x07
WRONG_OPERATION   = 0x08

# Słownik: kod błędu → czytelna nazwa
ERROR_NAMES = {
    OK_FRAME:          "OK_FRAME",
    WRONG_START_BYTE:  "WRONG_START_BYTE",
    WRONG_FRAME_ID:    "WRONG_FRAME_ID",
    WRONG_SLAVE_ID:    "WRONG_SLAVE_ID",
    WRONG_REGISTER:    "WRONG_REGISTER",
    WRONG_DATA_SIZE:   "WRONG_DATA_SIZE",
    WRONG_END_BYTE:    "WRONG_END_BYTE",
    WRONG_CRC_VALUE:   "WRONG_CRC_VALUE",
    WRONG_OPERATION:   "WRONG_OPERATION",
}

# ---------------------------------------------------------------------------
# Rejestry slave'a
# ---------------------------------------------------------------------------
REG_HOMING         = 0   # R/W  1B  – start/stop procesu homing
REG_MOVE_ANGLE     = 1   # R/W  4B  – kąt docelowy ruchu
REG_DIAG_CONTROL   = 2   # W    1B  – sterowanie diagnostyką
REG_DIAG_STATUS    = 3   # R    4B  – status diagnostyczny
REG_EMERGENCY_STOP = 4   # R/W  1B  – awaryjne zatrzymanie
REG_COUNT          = 5   # Liczba rejestrów (pierwszy nieprawidłowy indeks)

# Rozmiary rejestrów [bajty]
REG_SIZES = {
    REG_HOMING:         1,
    REG_MOVE_ANGLE:     4,
    REG_DIAG_CONTROL:   1,
    REG_DIAG_STATUS:    4,
    REG_EMERGENCY_STOP: 1,
}

# Czytelne nazwy rejestrów
REG_NAMES = {
    REG_HOMING:         "REG_HOMING",
    REG_MOVE_ANGLE:     "REG_MOVE_ANGLE",
    REG_DIAG_CONTROL:   "REG_DIAG_CONTROL",
    REG_DIAG_STATUS:    "REG_DIAG_STATUS",
    REG_EMERGENCY_STOP: "REG_EMERGENCY_STOP",
}

# Flagi uprawnień
REG_FLAGS_READ  = 0x02
REG_FLAGS_WRITE = 0x01

REG_PERMISSIONS = {
    REG_HOMING:         REG_FLAGS_READ | REG_FLAGS_WRITE,
    REG_MOVE_ANGLE:     REG_FLAGS_READ | REG_FLAGS_WRITE,
    REG_DIAG_CONTROL:   REG_FLAGS_WRITE,
    REG_DIAG_STATUS:    REG_FLAGS_READ,
    REG_EMERGENCY_STOP: REG_FLAGS_READ | REG_FLAGS_WRITE,
}

# ---------------------------------------------------------------------------
# Piny GPIO (BCM)
# ---------------------------------------------------------------------------
CS_PIN    = 5    # Chip Select – aktywny LOW
READY_PIN = 22   # READY_SLAVE_FLAG od STM32 – LOW = slave gotowy

# ---------------------------------------------------------------------------
# Timingowe stałe [sekundy]
# ---------------------------------------------------------------------------
DELAY_BETWEEN_STEPS    = 0.010   # 10 ms  – między krokami protokołu
CS_SETUP_TIME          = 0.0002  # 200 µs – setup CS przed transferem
INTER_TEST_DELAY       = 0.20    # 200 ms – przerwa między testami
READY_TIMEOUT_S        = 2.0     # Timeout oczekiwania na GPIO22
SPI_RESET_DELAY        = 0.10    # Opóźnienie przy resecie magistrali
DELAY_AFTER_VALIDATION = 0.020   # 20 ms  – po odebraniu VALIDATION_CODE
DELAY_AFTER_ERROR_VAL  = 0.050   # 50 ms  – po błędnej walidacji

# ---------------------------------------------------------------------------
# Parametry SPI
# ---------------------------------------------------------------------------
SPI_BUS       = 0
SPI_DEVICE    = 0
SPI_SPEED_HZ  = 500_000
SPI_MODE      = 0b00
