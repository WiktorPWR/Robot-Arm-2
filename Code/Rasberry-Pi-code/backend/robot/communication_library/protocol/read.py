"""
read.py – Pełna ścieżka READ (Slave → Master)
==============================================
Sekwencja kroków:

  1. Handshake        SYN → SYN_ACK → ACK
  2. Header           Master wysyła 14-bajtowy header (op=READ)
  3. Validation       Slave odsyła 1-bajtowy kod błędu
  4. Data + CRC       Slave wysyła payload + 4B CRC
  5. Master COMMIT    Master odsyła [START, COMMIT, err, END]

Po kroku 5 slave wykonuje wewnętrzny callback i wraca do WAIT_SYN.
"""

import struct
import time

from ..constants import (
    OK_FRAME, WRONG_CRC_VALUE,
    OPERATION_READ,
    DELAY_BETWEEN_STEPS, DELAY_AFTER_ERROR_VAL, DELAY_AFTER_VALIDATION,
)
from ..core.handshake import do_handshake
from .frame import build_header, build_master_commit, next_frame_id
from .crc   import crc32_stm32
from ..utils.log import CommunicationLog


def protocol_read(
    transport,
    register_address: int,
    expected_size:    int,
    *,
    frame_id:            int  | None = None,
    start_byte:          int  = 0x55,
    slave_id:            int  = 0x01,
    end_byte:            int  = 0xAA,
    corrupt_header_crc:  bool = False,
    expected_validation: int  = OK_FRAME,
    send_bad_commit:     bool = False,
    log: CommunicationLog | None = None,
) -> tuple[bool, bytes]:
    """
    Wykonuje pełną ścieżkę READ.

    Parametry
    ----------
    transport : SPITransport
        Warstwa transportowa (SPI + GPIO).
    register_address : int
        Adres rejestru do odczytu.
    expected_size : int
        Oczekiwana liczba bajtów danych (bez CRC) – zgodna z REG_SIZES[reg].
    frame_id : int | None
        Identyfikator ramki. Jeśli None – generowany automatycznie.
    start_byte : int
        Pozwala celowo wpisać błędny start_byte (testy protokołu).
    slave_id : int
        Pozwala celowo wpisać błędny slave_id (testy protokołu).
    end_byte : int
        Pozwala celowo wpisać błędny end_byte (testy protokołu).
    corrupt_header_crc : bool
        Jeśli True – CRC headera zostaje zepsute.
    expected_validation : int
        Oczekiwany kod walidacji (zwykle OK_FRAME).
    send_bad_commit : bool
        Jeśli True – master celowo wysyła błędny COMMIT (WRONG_CRC_VALUE).
        Używane do testowania obsługi błędów po stronie slave'a.
    log : CommunicationLog | None
        Opcjonalny obiekt logujący.

    Zwraca
    ------
    (success: bool, data: bytes)
        success – True jeśli odczyt zakończony sukcesem.
        data    – odebrane dane (bez CRC). Pusty bytes jeśli błąd.
    """
    if frame_id is None:
        frame_id = next_frame_id()
    if log is None:
        log = CommunicationLog()

    # ------------------------------------------------------------------ #
    # Krok 1: Handshake
    # ------------------------------------------------------------------ #
    if not do_handshake(transport, log):
        return False, b''

    # ------------------------------------------------------------------ #
    # Krok 2: Header
    # ------------------------------------------------------------------ #
    header = build_header(
        frame_id, OPERATION_READ, register_address, expected_size,
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
        return False, b''

    if validation != OK_FRAME:
        # Oczekiwany błąd – slave wraca do WAIT_SYN
        time.sleep(DELAY_AFTER_ERROR_VAL)
        return True, b''

    # ------------------------------------------------------------------ #
    # Krok 4: Data + CRC od slave (expected_size + 4 bajty)
    # ------------------------------------------------------------------ #
    raw = bytes(transport.xfer(
        [0x00] * (expected_size + 4),
        log,
        f"6R. Odbiór DATA+CRC ({expected_size + 4}B) ← slave",
    ))
    time.sleep(DELAY_BETWEEN_STEPS)

    payload_data = raw[:expected_size]
    crc_received = struct.unpack(">I", raw[-4:])[0]
    crc_calc     = crc32_stm32(payload_data)
    crc_ok       = (crc_received == crc_calc)

    commit_err = OK_FRAME if crc_ok else WRONG_CRC_VALUE

    # ------------------------------------------------------------------ #
    # Krok 5: COMMIT od mastera (4 bajty)
    # ------------------------------------------------------------------ #
    if send_bad_commit:
        commit_frame = build_master_commit(WRONG_CRC_VALUE)
    else:
        commit_frame = build_master_commit(commit_err)

    transport.xfer(list(commit_frame), log, "7R. COMMIT (4B) → slave")
    time.sleep(DELAY_BETWEEN_STEPS)

    if not crc_ok:
        return False, b''

    return True, bytes(payload_data)
