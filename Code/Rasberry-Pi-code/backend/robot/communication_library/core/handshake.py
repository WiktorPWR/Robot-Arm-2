"""
handshake.py – Faza nawiązania połączenia SYN / SYN_ACK / ACK
===============================================================
Protokół handshake (3 kroki):

  Master                     Slave
  ──────                     ─────
  SYN (0x97)    ──────────►
                             [RxCplt: ustawia DMA TX → SYN_ACK]
  0xFF (dummy)  ──────────►
                ◄──────────  SYN_ACK (0x98)
  ACK (0x99)    ──────────►
                             [Slave przechodzi do WAIT_HEADER]

Jeśli slave odpowie nieprawidłowym bajtem przy SYN_ACK, może być
zablokowany w poprzednim stanie (np. SEND_VALIDATION_CODE).
Funkcja `do_handshake` automatycznie ponawia po opóźnieniu.

Funkcja `recover_slave` służy do odblokowania slave'a który nie
odpowiada poprawnie na SYN – próbuje kilka razy z rosnącymi opóźnieniami.
"""

import time

from ..constants import (
    SYN_BYTE, SYN_ACK_BYTE, ACK_BYTE,
    DELAY_BETWEEN_STEPS, DELAY_AFTER_ERROR_VAL, DELAY_AFTER_VALIDATION,
)


def do_handshake(transport, log=None) -> bool:
    """
    Wykonuje pełny handshake: SYN → SYN_ACK → ACK.

    Parametry
    ----------
    transport : SPITransport
        Warstwa transportowa z metodą xfer().
    log : CommunicationLog | None
        Opcjonalny obiekt logujący wymianę bajtów.

    Zwraca
    ------
    bool
        True  – handshake zakończony sukcesem.
        False – slave nie odpowiedział poprawnym SYN_ACK (nawet po ponowieniu).
    """
    # Krok 1: Wyślij SYN
    transport.xfer([SYN_BYTE], log, "1. SYN → slave")
    # Daj STM32 czas na RxCpltCallback i przygotowanie DMA TX (SYN_ACK)
    time.sleep(DELAY_BETWEEN_STEPS)

    # Krok 2: Odbierz SYN_ACK (master wysyła dummy 0xFF)
    resp = transport.xfer([0xFF], log, "2. Odbiór SYN_ACK")
    time.sleep(DELAY_BETWEEN_STEPS)

    if resp[0] != SYN_ACK_BYTE:
        # Slave może być zablokowany w poprzednim stanie – daj mu czas na reset
        wait_s = DELAY_AFTER_ERROR_VAL * 2
        time.sleep(wait_s)

        # Ponów SYN
        transport.xfer([SYN_BYTE], log, "1b. SYN (ponowienie)")
        time.sleep(DELAY_BETWEEN_STEPS)
        resp = transport.xfer([0xFF], log, "2b. Odbiór SYN_ACK (ponowienie)")
        time.sleep(DELAY_BETWEEN_STEPS)

        if resp[0] != SYN_ACK_BYTE:
            return False

    # Krok 3: Wyślij ACK
    transport.xfer([ACK_BYTE], log, "3. ACK → slave")
    time.sleep(DELAY_BETWEEN_STEPS)
    return True


def recover_slave(transport, log=None, max_attempts: int = 3) -> bool:
    """
    Próbuje odblokować slave'a zablokowanego w stanie SEND_VALIDATION_CODE.

    Objaw blokady:
      Slave odpowiada starym bajtem (np. kodem błędu) zamiast SYN_ACK
      gdy master próbuje rozpocząć nową transakcję przez SYN.

    Procedura:
      1. Wyślij SYN i odbierz odpowiedź.
      2. Jeśli odpowiedź to SYN_ACK → uzupełnij handshake (ACK) i wróć True.
      3. W przeciwnym razie – odczekaj 100 ms i ponów (do max_attempts razy).

    Parametry
    ----------
    transport : SPITransport
    log : CommunicationLog | None
    max_attempts : int
        Liczba prób odblokowania.

    Zwraca
    ------
    bool
        True  – slave odblokowany i gotowy.
        False – nie udało się odblokować w zadanej liczbie prób.
    """
    if log is None:
        from ..utils.log import CommunicationLog
        log = CommunicationLog()

    for attempt in range(max_attempts):
        transport.xfer([SYN_BYTE], log, f"RECOVER: SYN próba {attempt + 1}")
        time.sleep(DELAY_AFTER_VALIDATION)   # Daj czas na ewentualny TxCplt

        resp = transport.xfer([0xFF], log, f"RECOVER: SYN_ACK próba {attempt + 1}")
        time.sleep(DELAY_BETWEEN_STEPS)

        if resp[0] == SYN_ACK_BYTE:
            # Slave odpowiedział – zakończ handshake
            transport.xfer([ACK_BYTE], log, "RECOVER: ACK")
            time.sleep(DELAY_BETWEEN_STEPS)
            return True

        # Slave nadal zablokowany – czekaj na wewnętrzny timeout STM32
        time.sleep(0.100)

    return False
