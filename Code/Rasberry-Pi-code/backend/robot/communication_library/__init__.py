"""
spi_stm32 – Biblioteka komunikacji SPI Master (Raspberry Pi ↔ STM32)
=====================================================================
Struktura pakietu:
    spi_stm32/
    ├── __init__.py          – publiczne API
    ├── constants.py         – wszystkie stałe protokołu
    ├── core/
    │   ├── __init__.py
    │   ├── transport.py     – niski poziom: SPI + GPIO (CS, READY_PIN)
    │   └── handshake.py     – faza SYN / SYN_ACK / ACK
    ├── protocol/
    │   ├── __init__.py
    │   ├── frame.py         – budowanie ramek (header, data+CRC, commit)
    │   ├── crc.py           – CRC-32 kompatybilne ze STM32 HAL
    │   ├── write.py         – pełna ścieżka WRITE
    │   └── read.py          – pełna ścieżka READ
    └── utils/
        ├── __init__.py
        ├── log.py           – CommunicationLog (MOSI/MISO dump)
        └── stats.py         – TestStats (liczniki pass/fail)

Szybki start:
    from spi_stm32 import SPIMaster

    spi = SPIMaster()
    spi.write(register=0, payload=bytes([0x01]))        # WRITE REG_HOMING
    ok, data = spi.read(register=3, size=4)             # READ REG_DIAG_STATUS
    spi.close()
"""

from .master import SPIMaster                       # noqa: F401 – główny interfejs
from .constants import (                            # noqa: F401 – stałe do importu
    REG_HOMING, REG_MOVE_ANGLE,
    REG_DIAG_CONTROL, REG_DIAG_STATUS, REG_EMERGENCY_STOP,
    OK_FRAME, WRONG_START_BYTE, WRONG_FRAME_ID, WRONG_SLAVE_ID,
    WRONG_REGISTER, WRONG_DATA_SIZE, WRONG_END_BYTE,
    WRONG_CRC_VALUE, WRONG_OPERATION,
)

__version__ = "1.0.0"
__all__ = ["SPIMaster"]
