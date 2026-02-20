"""
frame.py – Budowanie ramek protokołu SPI
=========================================
Protokół definiuje trzy rodzaje ramek:

1. HEADER (14 bajtów) – wysyłany przez mastera po handshake
   ┌────────┬──────────┬──────────┬───────────────┬──────────────────┬───────────┬──────────┬────────────┬──────────┐
   │ Offset │   Pole   │ Rozmiar  │     Typ        │     Wartość      │  Zakres   │          │            │          │
   ├────────┼──────────┼──────────┼───────────────┼──────────────────┼───────────┤          │            │          │
   │  [0]   │ start    │   1B     │ uint8          │ 0x55             │  fixed    │          │            │          │
   │  [1]   │ slave_id │   1B     │ uint8          │ 0x01             │  fixed    │          │            │          │
   │  [2:4] │ frame_id │   2B     │ uint16 BE      │ 1..0xFFFE        │  monot.   │          │            │          │
   │  [4]   │ op_type  │   1B     │ uint8          │ 0x01=R / 0x02=W  │           │          │            │          │
   │  [5:7] │ reg_addr │   2B     │ uint16 BE      │ 0..REG_COUNT-1   │           │          │            │          │
   │  [7:9] │ data_sz  │   2B     │ uint16 BE      │ 1..16            │           │          │            │          │
   │  [9:13]│ crc32    │   4B     │ uint32 BE      │ CRC z pól [1..8] │           │          │            │          │
   │  [13]  │ end      │   1B     │ uint8          │ 0xAA             │  fixed    │          │            │          │
   └────────┴──────────┴──────────┴───────────────┴──────────────────┴───────────┘

   CRC-32 jest liczone z 8 bajtów: slave_id + frame_id(2B) + op + addr(2B) + size(2B)

2. DATA + CRC (N+4 bajtów) – dane z dołączonym CRC-32
   Wysyłane przez mastera (WRITE) lub slave'a (READ).

3. COMMIT (4 bajty) – potwierdzenie odebranych danych
   Format: [START_BYTE, COMMIT_BYTE, error_code, END_BYTE]
   Wysyłane przez slave'a (po WRITE) lub mastera (po READ).
"""

import struct

from ..constants import (
    START_BYTE, SLAVE_ID, END_BYTE, COMMIT_BYTE,
    OPERATION_READ, OPERATION_WRITE, OK_FRAME,
)
from .crc import crc32_stm32


# ---------------------------------------------------------------------------
# Licznik frame_id (monotonicznie rosnący, 1..0xFFFE)
# ---------------------------------------------------------------------------
_frame_id_counter = 1


def next_frame_id() -> int:
    """
    Zwraca kolejny unikalny frame_id i inkrementuje globalny licznik.
    Zakres: 1..0xFFFE (0x0000 i 0xFFFF są zarezerwowane).
    """
    global _frame_id_counter
    fid = _frame_id_counter
    _frame_id_counter += 1
    if _frame_id_counter > 0xFFFE:
        _frame_id_counter = 1
    return fid


def reset_frame_id(start: int = 1) -> None:
    """Resetuje licznik frame_id (np. po restarcie STM32)."""
    global _frame_id_counter
    _frame_id_counter = max(1, min(start, 0xFFFE))


# ---------------------------------------------------------------------------
# Budowanie ramek
# ---------------------------------------------------------------------------

def build_header(
    frame_id:         int,
    operation_type:   int,
    register_address: int,
    data_size:        int,
    *,
    start_byte:       int  = START_BYTE,
    slave_id:         int  = SLAVE_ID,
    end_byte:         int  = END_BYTE,
    corrupt_crc:      bool = False,
) -> bytes:
    """
    Buduje 14-bajtowy header protokołu ze wbudowanym CRC-32.

    Format struct '>BBHBHHIB' (8 argumentów, 14 bajtów):
      B  start_byte       1B
      B  slave_id         1B
      H  frame_id         2B big-endian
      B  operation_type   1B
      H  register_address 2B big-endian
      H  data_size        2B big-endian
      I  crc32            4B big-endian
      B  end_byte         1B

    CRC jest obliczane z 8 bajtów:
      [slave_id, frame_id_hi, frame_id_lo, op, addr_hi, addr_lo, sz_hi, sz_lo]

    Parametry
    ----------
    frame_id : int
        Identyfikator ramki (1..0xFFFE). Slave odrzuca duplikaty.
    operation_type : int
        OPERATION_READ (0x01) lub OPERATION_WRITE (0x02).
    register_address : int
        Adres rejestru (0..REG_COUNT-1).
    data_size : int
        Rozmiar danych [bajty] – musi odpowiadać REG_SIZES[reg].
    start_byte : int
        Można nadpisać w testach błędów (domyślnie START_BYTE=0x55).
    slave_id : int
        Można nadpisać w testach błędów (domyślnie SLAVE_ID=0x01).
    end_byte : int
        Można nadpisać w testach błędów (domyślnie END_BYTE=0xAA).
    corrupt_crc : bool
        Jeśli True, XOR CRC z 0xDEADBEEF (test wykrywania błędów CRC).

    Zwraca
    ------
    bytes
        14-bajtowy header gotowy do wysłania przez xfer().
    """
    # 8 bajtów wejściowych do CRC
    crc_input = bytes([
        slave_id & 0xFF,
        (frame_id >> 8) & 0xFF,
        frame_id         & 0xFF,
        operation_type   & 0xFF,
        (register_address >> 8) & 0xFF,
        register_address         & 0xFF,
        (data_size >> 8) & 0xFF,
        data_size         & 0xFF,
    ])
    crc = crc32_stm32(crc_input)

    if corrupt_crc:
        crc = (crc ^ 0xDEADBEEF) & 0xFFFFFFFF

    return struct.pack(
        ">BBHBHHIB",
        start_byte        & 0xFF,
        slave_id          & 0xFF,
        frame_id          & 0xFFFF,
        operation_type    & 0xFF,
        register_address  & 0xFFFF,
        data_size         & 0xFFFF,
        crc,
        end_byte          & 0xFF,
    )


def build_data_with_crc(
    data:        bytes | list[int],
    corrupt_crc: bool = False,
) -> bytes:
    """
    Dołącza 4-bajtowe CRC-32 (big-endian) na koniec danych.

    Używane:
      • przez mastera przed wysłaniem payloadu (ścieżka WRITE)
      • przez slave'a przed wysłaniem odpowiedzi (ścieżka READ)

    Parametry
    ----------
    data : bytes | list[int]
        Właściwe dane do wysłania (bez CRC).
    corrupt_crc : bool
        Jeśli True, XOR CRC z 0xCAFEBABE (test wykrywania błędów CRC w danych).

    Zwraca
    ------
    bytes
        data + 4 bajty CRC (big-endian), gotowe do xfer().
    """
    raw = bytes(data)
    crc = crc32_stm32(raw)
    if corrupt_crc:
        crc = (crc ^ 0xCAFEBABE) & 0xFFFFFFFF
    return raw + struct.pack(">I", crc)


def build_master_commit(error_code: int = OK_FRAME) -> bytes:
    """
    Buduje 4-bajtową ramkę COMMIT wysyłaną przez mastera (ścieżka READ).

    Format: [START_BYTE, COMMIT_BYTE, error_code, END_BYTE]

    Parametry
    ----------
    error_code : int
        OK_FRAME (0x00) – dane poprawne.
        WRONG_CRC_VALUE  – slave powinien ponowić transmisję.

    Zwraca
    ------
    bytes
        4-bajtowy COMMIT.
    """
    return bytes([START_BYTE, COMMIT_BYTE, error_code & 0xFF, END_BYTE])


def parse_slave_commit(raw: bytes) -> tuple[bool, int]:
    """
    Parsuje 4-bajtowy COMMIT odebrany od slave'a (ścieżka WRITE).

    Parametry
    ----------
    raw : bytes
        4 bajty odebrane od slave'a.

    Zwraca
    ------
    (valid: bool, error_code: int)
        valid      – True jeśli struktura COMMIT jest poprawna.
        error_code – kod błędu z pola [2] (OK_FRAME = sukces).
    """
    if len(raw) < 4:
        return False, 0xFF
    ok = (raw[0] == START_BYTE and raw[1] == COMMIT_BYTE and raw[3] == END_BYTE)
    return ok, raw[2]
