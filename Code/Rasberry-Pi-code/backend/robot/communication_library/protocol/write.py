"""
write.py – Pełna ścieżka WRITE (Master → Slave)
=================================================
Sekwencja kroków:

  1. Handshake        SYN → SYN_ACK → ACK
  2. Header           Master wysyła 14-bajtowy header
  3. Validation       Slave odsyła 1-bajtowy kod błędu
  4. Data + CRC       Master wysyła payload + 4B CRC
  5. Slave COMMIT     Slave odsyła [START, COMMIT, err, END]
  6. (opcja) READY    Master czeka na GPIO22 LOW

Każdy krok używa wspólnych funkcji z transport.py i frame.py.
"""

import time
import struct

from ..constants import (
    OK_FRAME, WRONG_CRC_VALUE,
    OPERATION_WRITE,
    DELAY_BETWEEN_STEPS, DELAY_AFTER_ERROR_VAL, DELAY_AFTER_VALIDATION,
    READY_TIMEOUT_S,
)
from ..core.handshake import do_handshake
from .frame import build_header, build_data_with_crc, parse_slave_commit, next_frame_id
from ..utils.log import CommunicationLog


def protocol_write(
    transport,
    register_address: int,
    payload:          bytes | list[int],
    *,
    frame_id:          int  | None = None,
    start_byte:        int  = 0x55,
    slave_id:          int  = 0x01,
    end_byte:          int  = 0xAA,
    corrupt_header_crc: bool = False,
    corrupt_data_crc:   bool = False,
    expected_validation: int = OK_FRAME,
    wait_ready:        bool = True,
    ready_timeout:     float = READY_TIMEOUT_S,
    log:               CommunicationLog | None = None,
) -> tuple[bool, int]:
    """
    Wykonuje pełną ścieżkę WRITE.

    Parametry
    ----------
    transport : SPITransport
        Warstwa transportowa (SPI + GPIO).
    register_address : int
        Adres docelowego rejestru.
    payload : bytes | list[int]
        Dane do zapisania (bez CRC).
    frame_id : int | None
        Identyfikator ramki. Jeśli None – generowany automatycznie.
    start_byte : int
        Pozwala celowo wpisać błędny start_byte (testy protokołu).
    slave_id : int
        Pozwala celowo wpisać błędny slave_id (testy protokołu).
    end_byte : int
        Pozwala celowo wpisać błędny end_byte (testy protokołu).
    corrupt_header_crc : bool
        Jeśli True – CRC headera zostaje zepsute (testy protokołu).
    corrupt_data_crc : bool
        Jeśli True – CRC danych zostaje zepsute (testy protokołu).
    expected_validation : int
        Oczekiwany kod walidacji od slave'a. Zwykle OK_FRAME.
        Dla testów błędów np. WRONG_START_BYTE, WRONG_CRC_VALUE itp.
    wait_ready : bool
        Jeśli True – po pomyślnym WRITE czeka na GPIO22 LOW (callback slave'a).
    ready_timeout : float
        Timeout oczekiwania na GPIO22 [s].
    log : CommunicationLog | None
        Opcjonalny obiekt logujący wymianę bajtów.

    Zwraca
    ------
    (success: bool, slave_error_code: int)
        success          – True jeśli cała ścieżka zakończona sukcesem.
        slave_error_code – kod z pola COMMIT slave'a (OK_FRAME = brak błędów).
    """
    if frame_id is None:
        frame_id = next_frame_id()
    if log is None:
        log = CommunicationLog()
    payload = bytes(payload)

    # ------------------------------------------------------------------ #
    # Krok 1: Handshake
    # ------------------------------------------------------------------ #
    if not do_handshake(transport, log):
        return False, 0xFF

    # ------------------------------------------------------------------ #
    # Krok 2: Header
    # ------------------------------------------------------------------ #
    header = build_header(
        frame_id, OPERATION_WRITE, register_address, len(payload),
        start_byte=start_byte,
        slave_id=slave_id,
        end_byte=end_byte,
        corrupt_crc=corrupt_header_crc,
    )
    transport.xfer(list(header), log, f"4. HEADER ({len(header)}B) → slave")
    time.sleep(DELAY_BETWEEN_STEPS)

    # ------------------------------------------------------------------ #
    # Krok 3: Validation (1 bajt od slave)
    # ------------------------------------------------------------------ #
    resp = transport.xfer([0xFF], log, "5. Odbiór VALIDATION_CODE")
    time.sleep(DELAY_AFTER_VALIDATION)
    validation = resp[0]

    if validation != expected_validation:
        return False, validation

    if validation != OK_FRAME:
        # Oczekiwany błąd walidacji – slave wraca do WAIT_SYN
        time.sleep(DELAY_AFTER_ERROR_VAL)
        return True, validation   # Ścieżka błędu zatrzymana po walidacji

    # ------------------------------------------------------------------ #
    # Krok 4: Data + CRC
    # ------------------------------------------------------------------ #
    data_frame = build_data_with_crc(payload, corrupt_crc=corrupt_data_crc)
    transport.xfer(list(data_frame), log, f"6W. DATA+CRC ({len(data_frame)}B) → slave")
    time.sleep(DELAY_BETWEEN_STEPS)

    # ------------------------------------------------------------------ #
    # Krok 5: COMMIT od slave (4 bajty)
    # ------------------------------------------------------------------ #
    raw_commit = bytes(transport.xfer([0x00] * 4, log, "7W. Odbiór COMMIT od slave (4B)"))
    time.sleep(DELAY_BETWEEN_STEPS)

    valid_commit, slave_err = parse_slave_commit(raw_commit)
    if not valid_commit:
        return False, 0xFF

    if corrupt_data_crc:
        # Oczekujemy, że slave wykrył błąd CRC
        if slave_err != WRONG_CRC_VALUE:
            return False, slave_err
        return True, slave_err   # Sukces testu – slave wykrył błąd

    if slave_err != OK_FRAME:
        return False, slave_err

    # ------------------------------------------------------------------ #
    # Krok 6 (opcja): Czekaj na READY_SLAVE_FLAG
    # ------------------------------------------------------------------ #
    if wait_ready:
        transport.wait_ready(ready_timeout)

    return True, OK_FRAME
