"""
crc.py – CRC-32 kompatybilny ze sprzętowym CRC unit STM32 HAL
==============================================================
Algorytm (zweryfikowany empirycznie):

  Polynomial   : 0x04C11DB7  (standardowe CRC-32)
  Initialization: 0xFFFFFFFF
  Bit order    : przetwarzanie word-by-word, LITTLE-ENDIAN
  Padding      : dane uzupełniane zerami do wielokrotności 4B

Dlaczego LE (Little-Endian)?
  STM32 to architektura LE. Gdy HAL kopiuje dane do bufora uint32_t[],
  bajt[0] trafia na pozycję LSB wordu. CRC unit przetwarza każdy uint32_t
  od MSB do LSB – stąd potrzeba odczytania bajtów w kolejności LE.

Weryfikacja:
  Wejście (8B) : 01 00 11 02 00 00 00 01
  Oczekiwany   : 0xE202602A  (STM32 odpowiada OK_FRAME)
"""

import struct


def crc32_stm32(data: bytes | list[int]) -> int:
    """
    Oblicza CRC-32 kompatybilne ze sprzętowym CRC unit STM32 HAL.

    Parametry
    ----------
    data : bytes | list[int]
        Dane wejściowe. Muszą to być TYLKO pola używane do liczenia CRC
        (nie cały header – patrz frame.py).

    Zwraca
    ------
    int
        32-bitowa wartość CRC (unsigned).

    Przykład
    --------
    >>> crc32_stm32(bytes([0x01, 0x00, 0x11, 0x02, 0x00, 0x00, 0x00, 0x01]))
    0xE202602A
    """
    # Padding do wielokrotności 4 bajtów
    raw    = bytes(data)
    padded = raw + b'\x00' * ((4 - len(raw) % 4) % 4)

    # Interpretuj jako słowa 32-bit w kolejności LITTLE-ENDIAN
    n_words = len(padded) // 4
    words   = struct.unpack(f'<{n_words}I', padded)

    crc  = 0xFFFFFFFF
    poly = 0x04C11DB7

    for word in words:
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return crc


def verify_crc(data: bytes, received_crc: int) -> bool:
    """
    Sprawdza czy CRC obliczone z data zgadza się z received_crc.

    Parametry
    ----------
    data : bytes
        Dane (bez dołączonego CRC).
    received_crc : int
        CRC odebrane od slave'a / wpisane w ramce.

    Zwraca
    ------
    bool
        True jeśli CRC się zgadza.
    """
    return crc32_stm32(data) == received_crc
