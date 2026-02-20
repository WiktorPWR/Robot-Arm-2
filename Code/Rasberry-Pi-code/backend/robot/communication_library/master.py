"""
master.py – Fasada biblioteki: klasa SPIMaster
===============================================
SPIMaster to główny interfejs użytkownika biblioteki spi_stm32.
Łączy warstwę transportową (SPITransport) z logiką protokołu
(protocol_write, protocol_read) w prostym, wysokopoziomowym API.

Typowe użycie:
    from spi_stm32 import SPIMaster, REG_HOMING, REG_MOVE_ANGLE

    with SPIMaster() as spi:
        spi.write(REG_HOMING, bytes([0x01]))

        ok, angle_bytes = spi.read(REG_MOVE_ANGLE, size=4)
        if ok:
            import struct
            angle = struct.unpack(">I", angle_bytes)[0]
            print(f"Kąt: {angle}")
"""

import struct
from contextlib import contextmanager

from .constants import (
    REG_HOMING, REG_MOVE_ANGLE, REG_DIAG_CONTROL,
    REG_DIAG_STATUS, REG_EMERGENCY_STOP,
    REG_SIZES, REG_NAMES,
    OK_FRAME, READY_TIMEOUT_S,
    CS_PIN, READY_PIN, SPI_SPEED_HZ, SPI_MODE,
)
from .core.transport import SPITransport
from .core.handshake import recover_slave
from .protocol.write import protocol_write
from .protocol.read  import protocol_read
from .utils.log      import CommunicationLog


class SPIMaster:
    """
    Wysokopoziomowy interfejs do komunikacji SPI z STM32.

    Parametry
    ----------
    cs_pin : int
        BCM numer pinu Chip Select.
    ready_pin : int
        BCM numer pinu READY_SLAVE_FLAG.
    speed_hz : int
        Prędkość magistrali SPI [Hz].
    mode : int
        Tryb SPI (0–3).
    verbose : bool
        Jeśli True, drukuje informacje o każdej transakcji.

    Przykład
    --------
    spi = SPIMaster()
    ok, _ = spi.write(REG_HOMING, bytes([0x01]))
    ok, data = spi.read(REG_DIAG_STATUS, size=4)
    spi.close()

    # Lub jako context manager:
    with SPIMaster() as spi:
        spi.write_homing(1)
        ok, status = spi.read_diag_status()
    """

    def __init__(
        self,
        cs_pin:    int   = CS_PIN,
        ready_pin: int   = READY_PIN,
        speed_hz:  int   = SPI_SPEED_HZ,
        mode:      int   = SPI_MODE,
        verbose:   bool  = False,
    ):
        self._transport = SPITransport(
            cs_pin=cs_pin,
            ready_pin=ready_pin,
            speed_hz=speed_hz,
            mode=mode,
        )
        self.verbose = verbose

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self) -> None:
        """Zamyka SPI i zwalnia zasoby GPIO."""
        self._transport.close()

    # ------------------------------------------------------------------
    # Główne operacje – generyczne
    # ------------------------------------------------------------------

    def write(
        self,
        register:    int,
        payload:     bytes | list[int],
        *,
        wait_ready:  bool  = True,
        frame_id:    int   | None = None,
        log:         CommunicationLog | None = None,
        # Parametry testowe (nie używaj w produkcji)
        corrupt_header_crc: bool = False,
        corrupt_data_crc:   bool = False,
        expected_validation: int = OK_FRAME,
        start_byte:  int   = 0x55,
        slave_id:    int   = 0x01,
        end_byte:    int   = 0xAA,
    ) -> tuple[bool, int]:
        """
        Wykonuje operację WRITE na wskazanym rejestrze.

        Parametry
        ----------
        register : int
            Adres rejestru (np. REG_HOMING = 0).
        payload : bytes | list[int]
            Dane do zapisania. Rozmiar musi zgadzać się z REG_SIZES[register].
        wait_ready : bool
            Czeka na GPIO22 LOW po pomyślnym zapisie (sygnał z callbacku STM32).
        frame_id : int | None
            Ręczne ustawienie frame_id (None = auto-inkrementacja).
        log : CommunicationLog | None
            Opcjonalny obiekt logujący.
        corrupt_header_crc : bool
            (testy) Psuje CRC headera.
        corrupt_data_crc : bool
            (testy) Psuje CRC danych.
        expected_validation : int
            (testy) Oczekiwany kod błędu od slave'a.
        start_byte / slave_id / end_byte : int
            (testy) Nadpisanie stałych protokołu.

        Zwraca
        ------
        (success: bool, slave_error_code: int)
        """
        if self.verbose:
            reg_name = REG_NAMES.get(register, f"0x{register:04X}")
            print(f"[WRITE] reg={reg_name} size={len(bytes(payload))}B")

        return protocol_write(
            self._transport,
            register,
            payload,
            frame_id=frame_id,
            start_byte=start_byte,
            slave_id=slave_id,
            end_byte=end_byte,
            corrupt_header_crc=corrupt_header_crc,
            corrupt_data_crc=corrupt_data_crc,
            expected_validation=expected_validation,
            wait_ready=wait_ready,
            log=log,
        )

    def read(
        self,
        register:    int,
        size:        int | None = None,
        *,
        frame_id:    int | None = None,
        log:         CommunicationLog | None = None,
        # Parametry testowe
        corrupt_header_crc:  bool = False,
        expected_validation: int  = OK_FRAME,
        send_bad_commit:     bool = False,
        start_byte:  int = 0x55,
        slave_id:    int = 0x01,
        end_byte:    int = 0xAA,
    ) -> tuple[bool, bytes]:
        """
        Wykonuje operację READ z wskazanego rejestru.

        Parametry
        ----------
        register : int
            Adres rejestru.
        size : int | None
            Oczekiwana liczba bajtów danych.
            Jeśli None – odczytana automatycznie z REG_SIZES[register].
        frame_id : int | None
            Ręczne ustawienie frame_id.
        log : CommunicationLog | None
            Opcjonalny obiekt logujący.
        corrupt_header_crc : bool
            (testy) Psuje CRC headera.
        expected_validation : int
            (testy) Oczekiwany kod błędu od slave'a.
        send_bad_commit : bool
            (testy) Celowo wysyła błędny COMMIT.

        Zwraca
        ------
        (success: bool, data: bytes)
        """
        if size is None:
            size = REG_SIZES.get(register, 1)

        if self.verbose:
            reg_name = REG_NAMES.get(register, f"0x{register:04X}")
            print(f"[READ]  reg={reg_name} size={size}B")

        return protocol_read(
            self._transport,
            register,
            size,
            frame_id=frame_id,
            start_byte=start_byte,
            slave_id=slave_id,
            end_byte=end_byte,
            corrupt_header_crc=corrupt_header_crc,
            expected_validation=expected_validation,
            send_bad_commit=send_bad_commit,
            log=log,
        )

    # ------------------------------------------------------------------
    # Wygodne metody wysokopoziomowe (konkretne rejestry)
    # ------------------------------------------------------------------

    def write_homing(self, value: int) -> bool:
        """Ustawia REG_HOMING (0=stop, 1=start). Zwraca True jeśli OK."""
        ok, _ = self.write(REG_HOMING, bytes([value & 0xFF]))
        return ok

    def read_homing(self) -> tuple[bool, int]:
        """Odczytuje REG_HOMING. Zwraca (success, value)."""
        ok, data = self.read(REG_HOMING)
        return ok, data[0] if ok and data else 0

    def write_move_angle(self, angle: int) -> bool:
        """
        Ustawia REG_MOVE_ANGLE (uint32, big-endian).

        Parametry
        ----------
        angle : int
            Kąt docelowy (0..0xFFFFFFFF).
        """
        ok, _ = self.write(REG_MOVE_ANGLE, struct.pack(">I", angle & 0xFFFFFFFF))
        return ok

    def read_move_angle(self) -> tuple[bool, int]:
        """Odczytuje REG_MOVE_ANGLE. Zwraca (success, angle_uint32)."""
        ok, data = self.read(REG_MOVE_ANGLE)
        if ok and len(data) >= 4:
            return True, struct.unpack(">I", data)[0]
        return False, 0

    def write_diag_control(self, value: int) -> bool:
        """Zapisuje REG_DIAG_CONTROL (write-only). Zwraca True jeśli OK."""
        ok, _ = self.write(REG_DIAG_CONTROL, bytes([value & 0xFF]))
        return ok

    def read_diag_status(self) -> tuple[bool, int]:
        """Odczytuje REG_DIAG_STATUS (read-only, 4B). Zwraca (success, value)."""
        ok, data = self.read(REG_DIAG_STATUS)
        if ok and len(data) >= 4:
            return True, struct.unpack(">I", data)[0]
        return False, 0

    def set_emergency_stop(self, active: bool) -> bool:
        """Ustawia/kasuje REG_EMERGENCY_STOP. Zwraca True jeśli OK."""
        ok, _ = self.write(REG_EMERGENCY_STOP, bytes([0x01 if active else 0x00]))
        return ok

    def read_emergency_stop(self) -> tuple[bool, bool]:
        """Odczytuje stan REG_EMERGENCY_STOP. Zwraca (success, active)."""
        ok, data = self.read(REG_EMERGENCY_STOP)
        return ok, bool(data[0]) if ok and data else False

    # ------------------------------------------------------------------
    # Diagnostyka i narzędzia
    # ------------------------------------------------------------------

    def is_slave_ready(self) -> bool:
        """Zwraca True jeśli GPIO22 (READY_PIN) jest w stanie LOW."""
        return self._transport.is_ready()

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> bool:
        """Czeka na GPIO22 LOW. Zwraca True jeśli pin opadł przed timeout."""
        return self._transport.wait_ready(timeout)

    def recover(self, max_attempts: int = 3) -> bool:
        """
        Próbuje odblokować zablokowanego slave'a.
        Zwraca True jeśli slave odpowiada poprawnie po odzyskaniu.
        """
        return recover_slave(self._transport, max_attempts=max_attempts)

    def reset_bus(self) -> None:
        """Resetuje magistralę SPI (zamknięcie i ponowne otwarcie spidev)."""
        self._transport.reset()

    @property
    def transport(self) -> SPITransport:
        """Bezpośredni dostęp do warstwy transportowej (zaawansowane użycie)."""
        return self._transport
