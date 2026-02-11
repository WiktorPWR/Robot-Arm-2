"""
=============================================================================
  KOMPLEKSOWE TESTY PROTOKOŁU SPI - MASTER (Raspberry Pi -> STM32)
=============================================================================
  Plik: spi_master_tests.py

  Protokol (spi_slave_types.h):
        START_BYTE   = 0x55
        END_BYTE     = 0xAA
        SLAVE_ID     = 0x01
        SYN_BYTE     = 0x97
        SYN_ACK_BYTE = 0x98
        ACK_BYTE     = 0x99
        COMMIT_BYTE  = 0x9A

  Header (14 bajtow):
        [0]     start_byte       (1B)
        [1]     slave_id         (1B)
        [2:4]   frame_id         (2B big-endian)
        [4]     operation_type   (1B: 0x01=READ, 0x02=WRITE)
        [5:7]   register_address (2B big-endian)
        [7:9]   data_size        (2B big-endian)
        [9:13]  crc32            (4B big-endian) - CRC pol [1..8]
        [13]    end_byte         (1B)

  Frame_Errors_t:
        OK_FRAME          = 0x00
        WRONG_START_BYTE  = 0x01
        WRONG_FRAME_ID    = 0x02
        WRONG_SLAVE_ID    = 0x03
        WRONG_REGISTER    = 0x04
        WRONG_DATA_SIZE   = 0x05
        WRONG_END_BYTE    = 0x06
        WRONG_CRC_VALUE   = 0x07
        WRONG_OPERATION   = 0x08

  Rejestry:
        REG_HOMING         = 0  (R/W, 1B)
        REG_MOVE_ANGLE     = 1  (R/W, 4B)
        REG_DIAG_CONTROL   = 2  (W,   1B)
        REG_DIAG_STATUS    = 3  (R,   4B)
        REG_EMERGENCY_STOP = 4  (R/W, 1B)

  GPIO:
        CS_PIN    = 5   (chip select aktywny LOW)
        READY_PIN = 22  (READY_SLAVE_FLAG od STM32, LOW = gotowy)
=============================================================================
"""

import spidev
import RPi.GPIO as GPIO
import time
import struct
import sys
import os
from datetime import datetime

# =============================================================================
# KONFIGURACJA RAPORTU
# =============================================================================
REPORT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPORT_FILE = os.path.join(
    REPORT_DIR,
    f"spi_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
)

_report_lines = []


def rprint(*args, **kwargs):
    """Drukuje na ekran I zapisuje do bufora raportu."""
    line = " ".join(str(a) for a in args)
    print(line, **kwargs)
    _report_lines.append(line)


def save_report():
    """Zapisuje zebrany raport do pliku tekstowego."""
    try:
        with open(REPORT_FILE, "w", encoding="utf-8") as f:
            f.write("\n".join(_report_lines))
            f.write("\n")
        print(f"\n[RAPORT] Zapisano: {REPORT_FILE}")
    except Exception as e:
        print(f"\n[RAPORT] Blad zapisu: {e}")


# =============================================================================
# KONFIGURACJA GPIO
# =============================================================================
CS_PIN    = 5
READY_PIN = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN,    GPIO.OUT)
GPIO.output(CS_PIN,   GPIO.HIGH)
GPIO.setup(READY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# =============================================================================
# KONFIGURACJA SPI
# =============================================================================
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500_000
spi.mode         = 0b00
spi.no_cs        = True

# =============================================================================
# STALE PROTOKOLU
# =============================================================================
START_BYTE   = 0x55
SLAVE_ID     = 0x01
END_BYTE     = 0xAA
SYN_BYTE     = 0x97
SYN_ACK_BYTE = 0x98
ACK_BYTE     = 0x99
COMMIT_BYTE  = 0x9A

OPERATION_READ  = 0x01
OPERATION_WRITE = 0x02

OK_FRAME         = 0x00
WRONG_START_BYTE = 0x01
WRONG_FRAME_ID   = 0x02
WRONG_SLAVE_ID   = 0x03
WRONG_REGISTER   = 0x04
WRONG_DATA_SIZE  = 0x05
WRONG_END_BYTE   = 0x06
WRONG_CRC_VALUE  = 0x07
WRONG_OPERATION  = 0x08

ERROR_NAMES = {
    OK_FRAME:         "OK_FRAME",
    WRONG_START_BYTE: "WRONG_START_BYTE",
    WRONG_FRAME_ID:   "WRONG_FRAME_ID",
    WRONG_SLAVE_ID:   "WRONG_SLAVE_ID",
    WRONG_REGISTER:   "WRONG_REGISTER",
    WRONG_DATA_SIZE:  "WRONG_DATA_SIZE",
    WRONG_END_BYTE:   "WRONG_END_BYTE",
    WRONG_CRC_VALUE:  "WRONG_CRC_VALUE",
    WRONG_OPERATION:  "WRONG_OPERATION",
}

REG_HOMING         = 0
REG_MOVE_ANGLE     = 1
REG_DIAG_CONTROL   = 2
REG_DIAG_STATUS    = 3
REG_EMERGENCY_STOP = 4
REG_COUNT          = 5

REG_SIZES = {
    REG_HOMING:         1,
    REG_MOVE_ANGLE:     4,
    REG_DIAG_CONTROL:   1,
    REG_DIAG_STATUS:    4,
    REG_EMERGENCY_STOP: 1,
}

REG_NAMES = {
    REG_HOMING:         "REG_HOMING",
    REG_MOVE_ANGLE:     "REG_MOVE_ANGLE",
    REG_DIAG_CONTROL:   "REG_DIAG_CONTROL",
    REG_DIAG_STATUS:    "REG_DIAG_STATUS",
    REG_EMERGENCY_STOP: "REG_EMERGENCY_STOP",
}

REG_FLAGS_READ  = 0x02
REG_FLAGS_WRITE = 0x01
REG_PERMISSIONS = {
    REG_HOMING:         REG_FLAGS_READ | REG_FLAGS_WRITE,
    REG_MOVE_ANGLE:     REG_FLAGS_READ | REG_FLAGS_WRITE,
    REG_DIAG_CONTROL:   REG_FLAGS_WRITE,
    REG_DIAG_STATUS:    REG_FLAGS_READ,
    REG_EMERGENCY_STOP: REG_FLAGS_READ | REG_FLAGS_WRITE,
}

DELAY_BETWEEN_STEPS     = 0.010   # 10ms – czas miedzy krokami protokolu
CS_SETUP_TIME           = 0.0002  # 200us – czas setup CS (daje STM32 czas na DMA setup)
INTER_TEST_DELAY        = 0.20    # 200ms – przerwa miedzy testami
READY_TIMEOUT_S         = 2.0
SPI_RESET_DELAY         = 0.10

# Po odebraniu VALIDATION_CODE slave wykonuje TxCpltCallback ktory resetuje stan.
# Musimy odczekac az ten callback sie wykona zanim podniesiemy CS (nastepna transakcja).
# STM32 @ 72MHz: callback po ~kilku us, ale bezpiecznie dodajemy 20ms.
DELAY_AFTER_VALIDATION  = 0.020   # 20ms – czekaj po odebraniu VALIDATION_CODE

# Po bledie walidacji slave resetuje sie w TxCplt i czyści bufory.
# Dajemy mu czas zanim wystartujemy nowy test.
DELAY_AFTER_ERROR_VAL   = 0.050   # 50ms – dodatkowe czekanie po bledie walidacji

_frame_id_counter = 1


# =============================================================================
# LOGOWANIE I STATYSTYKI
# =============================================================================

class CommunicationLog:
    def __init__(self):
        self.transactions = []

    def add(self, step_name, mosi, miso):
        self.transactions.append({
            "step": step_name,
            "mosi": list(mosi),
            "miso": list(miso),
        })

    def print_flow(self):
        rprint("\n" + "╔" + "═" * 68 + "╗")
        rprint("║" + " " * 22 + "FLOW KOMUNIKACJI" + " " * 30 + "║")
        rprint("╚" + "═" * 68 + "╝")
        for i, t in enumerate(self.transactions, 1):
            rprint(f"\n{'─'*70}")
            rprint(f"  Krok {i}: {t['step']}")
            rprint(f"{'─'*70}")
            rprint(f"  MOSI (Master->Slave): {[f'0x{b:02X}' for b in t['mosi']]}")
            rprint(f"  MISO (Slave->Master): {[f'0x{b:02X}' for b in t['miso']]}")
        rprint("\n" + "=" * 70 + "\n")


class TestStats:
    def __init__(self):
        self.total    = 0
        self.passed   = 0
        self.failed   = 0
        self.categories = {}

    def add(self, category, passed):
        self.total += 1
        if passed:
            self.passed += 1
        else:
            self.failed += 1
        if category not in self.categories:
            self.categories[category] = {"passed": 0, "failed": 0}
        if passed:
            self.categories[category]["passed"] += 1
        else:
            self.categories[category]["failed"] += 1

    def print_summary(self):
        rprint("\n" + "=" * 70)
        rprint("  PODSUMOWANIE TESTOW")
        rprint("=" * 70)
        rprint(f"  Lacznie testow : {self.total}")
        rprint(f"  Udanych        : {self.passed}")
        rprint(f"  Nieudanych     : {self.failed}")
        if self.total > 0:
            rprint(f"  Wskaznik sukc. : {self.passed / self.total * 100:.1f}%")
        rprint("\n" + "-" * 70)
        rprint(f"  {'Kategoria':<38} {'Pass':>5} {'Total':>6} {'%':>7}")
        rprint("-" * 70)
        for cat, r in sorted(self.categories.items()):
            t   = r["passed"] + r["failed"]
            pct = r["passed"] / t * 100 if t else 0
            status = "OK " if r["failed"] == 0 else "!!!"
            rprint(f"  [{status}] {cat:<35} {r['passed']:>5} {t:>6} {pct:>6.1f}%")
        rprint("=" * 70)
        if self.failed == 0:
            rprint("  WSZYSTKIE TESTY PRZESZLY POMYSLNIE!")
        else:
            rprint(f"  UWAGA: {self.failed} testow wymaga uwagi")
        rprint("=" * 70 + "\n")


stats = TestStats()


# =============================================================================
# FUNKCJE POMOCNICZE
# =============================================================================

def err_name(code):
    return ERROR_NAMES.get(code, f"UNKNOWN(0x{code:02X})")


def next_frame_id():
    global _frame_id_counter
    fid = _frame_id_counter
    _frame_id_counter += 1
    if _frame_id_counter > 0xFFFE:
        _frame_id_counter = 1
    return fid


def crc32_stm32(data):
    """
    CRC-32 kompatybilne ze sprzętowym CRC unit STM32 HAL.

    Algorytm (zweryfikowany empirycznie programem spi_crc_finder.py):
      - Polynomial : 0x04C11DB7
      - Init       : 0xFFFFFFFF
      - Przetwarzanie: word-by-word LITTLE-ENDIAN
        STM32 jest architektura LE, wiec gdy HAL robi memcpy danych do bufora
        uint32_t[], bajty laduja w pamieci jako LE (bajt[0] = LSB wordu).
        CRC unit przetwarza kazdy uint32_t w kolejnosci: MSB -> LSB.
      - Padding: dane uzupelniane zerami do wielokrotnosci 4B.

    Przyklad weryfikacji (fid=0x0011):
      Dane wejsciowe (8B): 01 00 11 02 00 00 00 01
      Wyjscie CRC:         0xE202602A  (STM32 akceptuje OK_FRAME)
    """
    padded = bytes(data) + b'\x00' * ((4 - len(data) % 4) % 4)
    # little-endian: bajt[0] = LSB, bajt[3] = MSB w kazdym wordzie
    words  = struct.unpack(f'<{len(padded)//4}I', padded)
    crc    = 0xFFFFFFFF
    poly   = 0x04C11DB7
    for word in words:
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def build_header(
    frame_id,
    operation_type,
    register_address,
    data_size,
    start_byte=START_BYTE,
    slave_id=SLAVE_ID,
    end_byte=END_BYTE,
    corrupt_crc=False,
):
    """
    Buduje 14-bajtowy header protokolu ze wbudowanym CRC-32.

    Format struct '>BBHBHHIB':
        B  start_byte       (1B)
        B  slave_id         (1B)
        H  frame_id         (2B big-endian)
        B  operation_type   (1B)
        H  register_address (2B big-endian)
        H  data_size        (2B big-endian)
        I  crc32            (4B big-endian)
        B  end_byte         (1B)
    Razem: 1+1+2+1+2+2+4+1 = 14 bajtow, 8 argumentow.
    """
    # CRC liczymy z pol: slave_id + frame_id(2B) + op + addr(2B) + size(2B) = 8B
    crc_data = bytes([
        slave_id & 0xFF,
        (frame_id >> 8) & 0xFF, frame_id & 0xFF,
        operation_type & 0xFF,
        (register_address >> 8) & 0xFF, register_address & 0xFF,
        (data_size >> 8) & 0xFF, data_size & 0xFF,
    ])
    crc = crc32_stm32(crc_data)
    if corrupt_crc:
        crc = (crc ^ 0xDEADBEEF) & 0xFFFFFFFF

    # 8 argumentow dla 8 specyfikatorow w '>BBHBHHIB'
    return struct.pack(
        ">BBHBHHIB",
        start_byte & 0xFF,
        slave_id   & 0xFF,
        frame_id   & 0xFFFF,
        operation_type & 0xFF,
        register_address & 0xFFFF,
        data_size  & 0xFFFF,
        crc,
        end_byte   & 0xFF,
    )


def build_data_with_crc(data, corrupt_crc=False):
    """Dolicza 4-bajtowe CRC-32 na koniec danych."""
    data  = bytes(data)
    crc   = crc32_stm32(data)
    if corrupt_crc:
        crc = (crc ^ 0xCAFEBABE) & 0xFFFFFFFF
    return data + struct.pack(">I", crc)


def build_master_commit(error_code=OK_FRAME):
    """Buduje 4-bajtowa ramke COMMIT wysylana przez mastera (sciezka READ)."""
    return bytes([START_BYTE, COMMIT_BYTE, error_code & 0xFF, END_BYTE])


# =============================================================================
# SPI TRANSCEIVER
# =============================================================================

def spi_xfer(data, log=None, step=""):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(CS_SETUP_TIME)
    resp = spi.xfer2(list(data))
    GPIO.output(CS_PIN, GPIO.HIGH)
    if log is not None:
        log.add(step, data, resp)
    return resp


def wait_for_ready(timeout=READY_TIMEOUT_S):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if GPIO.input(READY_PIN) == GPIO.LOW:
            return True
        time.sleep(0.001)
    return False


def reset_spi():
    global spi
    rprint("    [RESET SPI] Czyszczenie magistrali...")
    try:
        spi.close()
    except Exception:
        pass
    time.sleep(SPI_RESET_DELAY)
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500_000
    spi.mode         = 0b00
    spi.no_cs        = True
    try:
        for _ in range(3):
            spi.xfer2([0x00] * 16)
    except Exception:
        pass
    rprint("    [RESET SPI] Gotowy.\n")


# =============================================================================
# KROKI PROTOKOLU
# =============================================================================

def do_handshake(log):
    """
    Wykonuje faze handshake: SYN -> SYN_ACK -> ACK.

    Jesli slave jest zablokowany (odpowiada garbage przy SYN zamiast byc idle),
    czeka na DELAY_AFTER_VALIDATION i ponawia. Slave odblokuje sie po TxCpltCallback.
    """
    # Krok 1: SYN
    spi_xfer([SYN_BYTE], log, "1. SYN -> slave")
    # Daj czas slave'owi na RxCpltCallback (odebranie SYN) i setup DMA transmit
    time.sleep(DELAY_BETWEEN_STEPS)

    # Krok 2: odbior SYN_ACK
    resp = spi_xfer([0xFF], log, "2. Odbior SYN_ACK")
    time.sleep(DELAY_BETWEEN_STEPS)

    if resp[0] != SYN_ACK_BYTE:
        # Slave moze byc zablokowany w poprzednim stanie.
        # Daj mu czas na wewnetrzny timer reset (1s timeout STM32) lub TxCplt.
        rprint(f"    [HS] SYN_ACK niepoprawny (0x{resp[0]:02X}), "
               f"czekam {DELAY_AFTER_ERROR_VAL*2*1000:.0f}ms na reset slave...")
        time.sleep(DELAY_AFTER_ERROR_VAL * 2)

        # Ponow SYN
        spi_xfer([SYN_BYTE], log, "1b. SYN (ponownie)")
        time.sleep(DELAY_BETWEEN_STEPS)
        resp = spi_xfer([0xFF], log, "2b. Odbior SYN_ACK (ponownie)")
        time.sleep(DELAY_BETWEEN_STEPS)

        if resp[0] != SYN_ACK_BYTE:
            rprint(f"    [HS] FAIL: SYN_ACK nadal niepoprawny: 0x{resp[0]:02X}")
            return False
        rprint(f"    [HS] Slave odblokowany, SYN_ACK OK po ponowieniu")

    # Krok 3: ACK
    spi_xfer([ACK_BYTE], log, "3. ACK -> slave")
    time.sleep(DELAY_BETWEEN_STEPS)
    return True


def do_send_header(header, log):
    spi_xfer(list(header), log, f"4. HEADER ({len(header)}B) -> slave")
    time.sleep(DELAY_BETWEEN_STEPS)


def do_recv_validation(log):
    """
    Odbiera 1-bajtowy kod walidacji od slave.

    WAZNE: Po tym transferze slave wykonuje HAL_SPI_TxCpltCallback ktory:
      - jesli OK_FRAME:  przechodzi do WAIT_DATA lub SEND_DATA
      - jesli blad:      wywoluje clear_communication_buffers() i wraca do WAIT_SYN

    Master musi odczekac az ten callback sie wykona ZANIM wykona kolejna
    transakcje SPI (CS toggle). Inaczej HAL_SPI_DMAStop() w GPIO callback
    anuluje TxCplt i slave zostaje zablokowany w SEND_VALIDATION_CODE.
    """
    resp = spi_xfer([0xFF], log, "5. Odbior VALIDATION_CODE")
    # Kluczowe opoznienie: daj STM32 czas na wykonanie TxCpltCallback
    time.sleep(DELAY_AFTER_VALIDATION)
    return resp[0]


def slave_recover(log=None, max_attempts=3):
    """
    Wykrywa i naprawia zablokowany stan slave.

    Slave moze zostac zablokowany w SEND_VALIDATION_CODE jesli master zbyt szybko
    zakonczyl poprzednia transakcje. Objaw: przy SYN slave odpowiada starym
    VALIDATION_CODE zamiast byc idle.

    Procedura odzyskiwania:
    1. Wyslij SYN i sprawdz czy odpowiedz to SYN_ACK
    2. Jesli nie, slave jest zablokowany - wysylaj dummy bajty az sie odblokuje
    3. Ponow SYN

    Zwraca True jesli slave jest gotowy (odpowiada na SYN).
    """
    if log is None:
        log = CommunicationLog()

    for attempt in range(max_attempts):
        # Probujemy SYN
        resp_syn = spi_xfer([SYN_BYTE], log, f"RECOVER: SYN proba {attempt+1}")
        time.sleep(DELAY_AFTER_VALIDATION)  # czekaj na ewentualny TxCplt

        # Sprawdz czy slave jest juz w WAIT_SYN (odpowie SYN_ACK dopiero po kolejnym CS)
        resp_poll = spi_xfer([0xFF], log, f"RECOVER: poll SYN_ACK proba {attempt+1}")
        time.sleep(DELAY_BETWEEN_STEPS)

        if resp_poll[0] == SYN_ACK_BYTE:
            # Slave odpowiedzial SYN_ACK - jest gotowy
            # Dokoncz handshake: wyslij ACK
            spi_xfer([ACK_BYTE], log, "RECOVER: ACK")
            time.sleep(DELAY_BETWEEN_STEPS)
            return True

        # Slave zablokowany - daj mu czas na timeout wewnetrzny
        rprint(f"    [RECOVER] Slave zablokowany (MISO=0x{resp_poll[0]:02X}), "
               f"czekam 100ms... proba {attempt+1}/{max_attempts}")
        time.sleep(0.100)

    rprint("    [RECOVER] Nie udalo sie odblokowac slave!")
    return False


def do_write_data(data_with_crc, log):
    spi_xfer(list(data_with_crc), log,
             f"6W. DATA+CRC ({len(data_with_crc)}B) -> slave")
    time.sleep(DELAY_BETWEEN_STEPS)


def do_recv_slave_commit(log):
    resp = spi_xfer([0x00] * 4, log, "7W. Odbior COMMIT od slave (4B)")
    time.sleep(DELAY_BETWEEN_STEPS)
    return bytes(resp)


def do_recv_data(size, log):
    resp = spi_xfer([0x00] * size, log,
                    f"6R. Odbior DATA+CRC od slave ({size}B)")
    time.sleep(DELAY_BETWEEN_STEPS)
    return bytes(resp)


def do_send_master_commit(commit_frame, log):
    spi_xfer(list(commit_frame), log, "7R. COMMIT (4B) -> slave")
    time.sleep(DELAY_BETWEEN_STEPS)


# =============================================================================
# FORMATOWANIE WYJSCIA
# =============================================================================

def _section(title):
    rprint("\n\n" + "=" * 70)
    rprint(f"  {title}")
    rprint("=" * 70)


def _test_header(name, category, details=""):
    rprint(f"\n{'─'*70}")
    rprint(f"TEST: {name}")
    rprint(f"      Kat.: {category}  |  {details}")
    rprint(f"{'─'*70}")


def _print_val(val, expected):
    ok = "OK  " if val == expected else "FAIL"
    rprint(f"    [{ok}] VALIDATION: {err_name(val)} (0x{val:02X})"
           f"  [oczekiwano: {err_name(expected)}]")


def _print_commit_bytes(commit, label="COMMIT"):
    if len(commit) >= 4:
        s = "OK" if commit[0] == START_BYTE  else "!!"
        c = "OK" if commit[1] == COMMIT_BYTE else "!!"
        e = "OK" if commit[3] == END_BYTE    else "!!"
        rprint(f"    {label}: START[{s}]=0x{commit[0]:02X}  "
               f"COMMIT[{c}]=0x{commit[1]:02X}  "
               f"ERR=0x{commit[2]:02X}({err_name(commit[2])})  "
               f"END[{e}]=0x{commit[3]:02X}")


# =============================================================================
# PROTOKOL WRITE
# =============================================================================

def protocol_write(
    category,
    test_name,
    register_address,
    payload,
    frame_id=None,
    start_byte=START_BYTE,
    slave_id=SLAVE_ID,
    end_byte=END_BYTE,
    corrupt_header_crc=False,
    corrupt_data_crc=False,
    expected_validation=OK_FRAME,
    wait_ready=True,
    verbose=True,
):
    """
    Pelny przebieg WRITE:
        Handshake -> Header -> Validation -> Data+CRC -> COMMIT(slave)
        -> opcjonalnie czeka na READY_SLAVE_FLAG (GPIO22)
    """
    if frame_id is None:
        frame_id = next_frame_id()

    log    = CommunicationLog()
    passed = False

    _test_header(
        test_name, category,
        f"WRITE reg={REG_NAMES.get(register_address, f'0x{register_address:04X}')}"
        f" fid=0x{frame_id:04X} data={len(payload)}B"
    )

    try:
        # --- Handshake ---
        if not do_handshake(log):
            rprint("    FAIL: Handshake nieudany")
            stats.add(category, False)
            return False

        # --- Header ---
        header = build_header(
            frame_id, OPERATION_WRITE, register_address, len(payload),
            start_byte=start_byte, slave_id=slave_id, end_byte=end_byte,
            corrupt_crc=corrupt_header_crc,
        )
        do_send_header(header, log)

        # --- Walidacja ---
        val = do_recv_validation(log)
        _print_val(val, expected_validation)

        if val != expected_validation:
            rprint(f"    FAIL: oczekiwano {err_name(expected_validation)}, "
                   f"otrzymano {err_name(val)}")
            stats.add(category, False)
            return False

        if val != OK_FRAME:
            # Oczekiwany blad walidacji – test zaliczony
            # Czekaj az slave wykona TxCpltCallback i wróci do WAIT_SYN
            time.sleep(DELAY_AFTER_ERROR_VAL)
            rprint(f"    OK: Slave odrzucil naglowek: {err_name(val)}")
            rprint("  PASSED\n")
            stats.add(category, True)
            return True

        # --- Dane z CRC ---
        data_frame = build_data_with_crc(payload, corrupt_crc=corrupt_data_crc)
        do_write_data(data_frame, log)

        # --- COMMIT od slave ---
        commit = do_recv_slave_commit(log)
        _print_commit_bytes(commit, "Slave COMMIT")

        if len(commit) < 4:
            rprint("    FAIL: COMMIT zbyt krotki")
            stats.add(category, False)
            return False

        if not (commit[0] == START_BYTE and
                commit[1] == COMMIT_BYTE and
                commit[3] == END_BYTE):
            rprint(f"    FAIL: Bledna struktura COMMIT: {[f'0x{b:02X}' for b in commit]}")
            stats.add(category, False)
            return False

        slave_err = commit[2]

        if corrupt_data_crc:
            if slave_err != WRONG_CRC_VALUE:
                rprint(f"    FAIL: Oczekiwano WRONG_CRC_VALUE od slave, "
                       f"otrzymano 0x{slave_err:02X}")
                stats.add(category, False)
                return False
            rprint(f"    OK: Slave wykryl blad CRC danych: {err_name(slave_err)}")
        else:
            if slave_err != OK_FRAME:
                rprint(f"    FAIL: Slave zglosil blad danych: {err_name(slave_err)}")
                stats.add(category, False)
                return False

        # --- Czekaj na READY_SLAVE_FLAG ---
        if wait_ready and slave_err == OK_FRAME and not corrupt_data_crc:
            if verbose:
                rprint("    Czekam na READY_SLAVE_FLAG (GPIO22 LOW)...")
            ready = wait_for_ready()
            if verbose:
                if ready:
                    rprint("    OK: GPIO22 LOW – slave gotowy")
                else:
                    rprint("    WARN: Timeout GPIO22 – callback moze nie dzialac")

        rprint("  PASSED\n")
        passed = True
        stats.add(category, True)
        return True

    except Exception as e:
        rprint(f"    EXCEPTION: {e}")
        import traceback
        rprint(traceback.format_exc())
        stats.add(category, False)
        return False
    finally:
        if not passed:
            log.print_flow()


# =============================================================================
# PROTOKOL READ
# =============================================================================

def protocol_read(
    category,
    test_name,
    register_address,
    expected_size,
    frame_id=None,
    start_byte=START_BYTE,
    slave_id=SLAVE_ID,
    end_byte=END_BYTE,
    corrupt_header_crc=False,
    expected_validation=OK_FRAME,
    send_bad_commit=False,
    verbose=True,
):
    """
    Pelny przebieg READ:
        Handshake -> Header -> Validation -> DATA+CRC(slave) -> COMMIT(master)
    Zwraca (sukces, dane_bez_crc).
    """
    if frame_id is None:
        frame_id = next_frame_id()

    log         = CommunicationLog()
    passed      = False
    result_data = b''

    _test_header(
        test_name, category,
        f"READ reg={REG_NAMES.get(register_address, f'0x{register_address:04X}')}"
        f" fid=0x{frame_id:04X} size={expected_size}B"
    )

    try:
        # --- Handshake ---
        if not do_handshake(log):
            rprint("    FAIL: Handshake nieudany")
            stats.add(category, False)
            return False, b''

        # --- Header ---
        header = build_header(
            frame_id, OPERATION_READ, register_address, expected_size,
            start_byte=start_byte, slave_id=slave_id, end_byte=end_byte,
            corrupt_crc=corrupt_header_crc,
        )
        do_send_header(header, log)

        # --- Walidacja ---
        val = do_recv_validation(log)
        _print_val(val, expected_validation)

        if val != expected_validation:
            rprint(f"    FAIL: oczekiwano {err_name(expected_validation)}, "
                   f"otrzymano {err_name(val)}")
            stats.add(category, False)
            return False, b''

        if val != OK_FRAME:
            rprint(f"    OK: Slave odrzucil naglowek: {err_name(val)}")
            # Czekaj az slave wykona TxCpltCallback i wróci do WAIT_SYN
            time.sleep(DELAY_AFTER_ERROR_VAL)
            rprint("  PASSED\n")
            stats.add(category, True)
            return True, b''

        # --- Odbiór danych + CRC od slave ---
        raw = do_recv_data(expected_size + 4, log)

        payload_data = raw[:expected_size]
        crc_rx       = struct.unpack(">I", raw[-4:])[0]
        crc_calc     = crc32_stm32(payload_data)

        if verbose:
            rprint(f"    Dane od slave : {[f'0x{b:02X}' for b in payload_data]}")
            rprint(f"    CRC rx=0x{crc_rx:08X}  calc=0x{crc_calc:08X}  "
                   f"{'OK' if crc_rx == crc_calc else 'MISMATCH'}")

        commit_err = OK_FRAME if (crc_rx == crc_calc) else WRONG_CRC_VALUE

        # --- COMMIT od mastera ---
        if send_bad_commit:
            commit_frame = build_master_commit(WRONG_CRC_VALUE)
            rprint("    WARN: Wysylam celowo bledny COMMIT (WRONG_CRC_VALUE)")
        else:
            commit_frame = build_master_commit(commit_err)
        do_send_master_commit(commit_frame, log)

        if commit_err != OK_FRAME:
            rprint(f"    FAIL: CRC mismatch w danych od slave")
            stats.add(category, False)
            return False, b''

        result_data = bytes(payload_data)
        rprint("  PASSED\n")
        passed = True
        stats.add(category, True)
        return True, result_data

    except Exception as e:
        rprint(f"    EXCEPTION: {e}")
        import traceback
        rprint(traceback.format_exc())
        stats.add(category, False)
        return False, b''
    finally:
        if not passed:
            log.print_flow()


# =============================================================================
# PROTOKOL TIMEOUT TEST
# =============================================================================

def protocol_timeout_test(category, test_name, abort_after, delay_s=2.5):
    """
    Testuje reset STM32 po timeoucie.
    abort_after in {'syn', 'ack', 'header', 'data'}
    """
    log    = CommunicationLog()
    passed = False

    _test_header(test_name, category,
                 f"abort_after='{abort_after}'  delay={delay_s}s")

    try:
        # SYN
        spi_xfer([SYN_BYTE], log, "1. SYN")
        if abort_after == 'syn':
            rprint(f"    Czekam {delay_s}s (symulacja timeoutu po SYN)...")
            time.sleep(delay_s)
            reset_spi()
            return _verify_slave_reset(log, category)

        time.sleep(DELAY_BETWEEN_STEPS)

        # SYN_ACK
        resp = spi_xfer([0xFF], log, "2. SYN_ACK")
        if resp[0] != SYN_ACK_BYTE:
            rprint("    FAIL: Brak SYN_ACK")
            stats.add(category, False)
            return False
        time.sleep(DELAY_BETWEEN_STEPS)

        # ACK
        spi_xfer([ACK_BYTE], log, "3. ACK")
        if abort_after == 'ack':
            rprint(f"    Czekam {delay_s}s (symulacja timeoutu po ACK)...")
            time.sleep(delay_s)
            reset_spi()
            return _verify_slave_reset(log, category)

        time.sleep(DELAY_BETWEEN_STEPS)

        # Header (poprawny)
        fid    = next_frame_id()
        header = build_header(fid, OPERATION_WRITE, REG_HOMING, 1)
        do_send_header(header, log)
        if abort_after == 'header':
            rprint(f"    Czekam {delay_s}s (symulacja timeoutu po HEADER)...")
            time.sleep(delay_s)
            reset_spi()
            return _verify_slave_reset(log, category)

        # Validation
        val = do_recv_validation(log)
        if val != OK_FRAME:
            rprint(f"    FAIL: Niespodziewany blad walidacji: {err_name(val)}")
            stats.add(category, False)
            return False

        # Data
        data_frame = build_data_with_crc(bytes([0x01]))
        do_write_data(data_frame, log)
        if abort_after == 'data':
            rprint(f"    Czekam {delay_s}s (symulacja timeoutu po DATA)...")
            time.sleep(delay_s)
            reset_spi()
            return _verify_slave_reset(log, category)

        rprint("    FAIL: Kod testu nie powinien tu dotrzec")
        stats.add(category, False)
        return False

    except Exception as e:
        rprint(f"    EXCEPTION: {e}")
        import traceback
        rprint(traceback.format_exc())
        reset_spi()
        stats.add(category, False)
        return False
    finally:
        if not passed:
            log.print_flow()


def _verify_slave_reset(log, category):
    """Po timeoucie sprawdza czy STM32 zresetowal sie (odpowiada na SYN)."""
    try:
        # 1. Wysyłamy SYN. 
        # Ważne: W SPI odbieramy dane w tym samym momencie, w którym wysyłamy.
        # Po timeoutcie Slave może odpowiedzieć SYN_ACK (0x98) natychmiast.
        resp1 = spi_xfer([SYN_BYTE], log, "SYN (po timeout)")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 2. Wysyłamy dummy byte, aby odebrać SYN_ACK, jeśli nie przyszedł wcześniej
        resp2 = spi_xfer([0xFF], log, "SYN_ACK (po timeout)")
        
        # SPRAWDZENIE:
        # Akceptujemy SYN_ACK (0x98) w dowolnym z tych dwóch kroków.
        # To rozwiązuje problem przesunięcia o 1 bajt po resecie.
        if resp1[0] == SYN_ACK_BYTE or resp2[0] == SYN_ACK_BYTE:
            rprint("    OK: STM32 zresetowal sie i odpowiada na SYN")
            rprint("  PASSED\n")
            stats.add(category, True)
            return True
        else:
            # Wypisz co faktycznie odebraliśmy dla debugu
            rprint(f"    FAIL: STM32 nie odpowiedzial SYN_ACK. Odebrano: 1:[0x{resp1[0]:02X}], 2:[0x{resp2[0]:02X}]")
            stats.add(category, False)
            return False
            
    except Exception as e:
        rprint(f"    EXCEPTION: {e}")
        stats.add(category, False)
        return False


# =============================================================================
# BATERIA TESTOW
# =============================================================================

def run_all_tests():
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    rprint("\n" + "=" * 70)
    rprint(f"  TESTY PROTOKOLU SPI  |  Start: {ts}")
    rprint("=" * 70)

    # =========================================================================
    # KAT 1 – POPRAWNY ZAPIS (WRITE)
    # =========================================================================
    _section("KATEGORIA 1: Poprawny zapis (WRITE) do rejestrow")

    protocol_write("01-Write-Valid", "1.1  WRITE REG_HOMING = 1 (start homing)",
                   REG_HOMING, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.2  WRITE REG_HOMING = 0 (stop homing)",
                   REG_HOMING, bytes([0x00]))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.3  WRITE REG_MOVE_ANGLE = 90",
                   REG_MOVE_ANGLE, struct.pack(">I", 90))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.4  WRITE REG_MOVE_ANGLE = 0",
                   REG_MOVE_ANGLE, struct.pack(">I", 0))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.5  WRITE REG_MOVE_ANGLE = 0xFFFFFFFF",
                   REG_MOVE_ANGLE, struct.pack(">I", 0xFFFFFFFF))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.6  WRITE REG_DIAG_CONTROL = 0x01",
                   REG_DIAG_CONTROL, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.7  WRITE REG_EMERGENCY_STOP = 1 (aktywacja)",
                   REG_EMERGENCY_STOP, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    protocol_write("01-Write-Valid", "1.8  WRITE REG_EMERGENCY_STOP = 0 (dezaktywacja)",
                   REG_EMERGENCY_STOP, bytes([0x00]))
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 2 – POPRAWNY ODCZYT (READ)
    # =========================================================================
    _section("KATEGORIA 2: Poprawny odczyt (READ) z rejestrów")

    ok, data = protocol_read("02-Read-Valid", "2.1  READ REG_HOMING (1B)",
                              REG_HOMING, REG_SIZES[REG_HOMING])
    if ok:
        rprint(f"    -> REG_HOMING = 0x{data[0]:02X}")
    time.sleep(INTER_TEST_DELAY)

    ok, data = protocol_read("02-Read-Valid", "2.2  READ REG_MOVE_ANGLE (4B)",
                              REG_MOVE_ANGLE, REG_SIZES[REG_MOVE_ANGLE])
    if ok:
        rprint(f"    -> REG_MOVE_ANGLE = {struct.unpack('>I', data)[0]}")
    time.sleep(INTER_TEST_DELAY)

    ok, data = protocol_read("02-Read-Valid", "2.3  READ REG_DIAG_STATUS (4B, read-only)",
                              REG_DIAG_STATUS, REG_SIZES[REG_DIAG_STATUS])
    if ok:
        rprint(f"    -> REG_DIAG_STATUS = 0x{struct.unpack('>I', data)[0]:08X}")
    time.sleep(INTER_TEST_DELAY)

    ok, data = protocol_read("02-Read-Valid", "2.4  READ REG_EMERGENCY_STOP (1B)",
                              REG_EMERGENCY_STOP, REG_SIZES[REG_EMERGENCY_STOP])
    if ok:
        rprint(f"    -> REG_EMERGENCY_STOP = 0x{data[0]:02X}")
    time.sleep(INTER_TEST_DELAY)

    # Round-trip: WRITE potem READ
    test_val = 0x12345678
    protocol_write("02-Read-Valid", "2.5a WRITE REG_MOVE_ANGLE = 0x12345678",
                   REG_MOVE_ANGLE, struct.pack(">I", test_val))
    time.sleep(INTER_TEST_DELAY)
    ok, data = protocol_read("02-Read-Valid", "2.5b READ REG_MOVE_ANGLE (round-trip)",
                              REG_MOVE_ANGLE, REG_SIZES[REG_MOVE_ANGLE])
    if ok:
        rd = struct.unpack(">I", data)[0]
        match = "OK" if rd == test_val else "FAIL"
        rprint(f"    -> Round-trip [{match}]: "
               f"zapisano=0x{test_val:08X}  odczytano=0x{rd:08X}")
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 3 – BLEDNY START_BYTE
    # =========================================================================
    _section("KATEGORIA 3: Bledny START_BYTE w naglowku")

    for bad, label in [(0xAA, "0xAA"), (0x00, "0x00"),
                       (0xFF, "0xFF"), (0x56, "0x56")]:
        protocol_write("03-Bad-StartByte",
                       f"3.x  WRITE z START_BYTE={label}",
                       REG_HOMING, bytes([0x01]),
                       start_byte=bad,
                       expected_validation=WRONG_START_BYTE)
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 4 – BLEDNY SLAVE_ID
    # =========================================================================
    _section("KATEGORIA 4: Bledny SLAVE_ID w naglowku")

    for bad, label in [(0x02, "0x02"), (0x00, "0x00"), (0xFF, "0xFF")]:
        protocol_write("04-Bad-SlaveID",
                       f"4.x  WRITE z SLAVE_ID={label}",
                       REG_HOMING, bytes([0x01]),
                       slave_id=bad,
                       expected_validation=WRONG_SLAVE_ID)
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 5 – BLEDNY END_BYTE
    # =========================================================================
    _section("KATEGORIA 5: Bledny END_BYTE w naglowku")

    for bad, label in [(0x55, "0x55"), (0x00, "0x00"), (0xFF, "0xFF")]:
        protocol_write("05-Bad-EndByte",
                       f"5.x  WRITE z END_BYTE={label}",
                       REG_HOMING, bytes([0x01]),
                       end_byte=bad,
                       expected_validation=WRONG_END_BYTE)
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 6 – BLEDNY ADRES REJESTRU
    # =========================================================================
    _section("KATEGORIA 6: Bledny adres rejestru (WRONG_REGISTER)")

    for bad, label in [(REG_COUNT, str(REG_COUNT)), (0x10, "0x10"),
                       (0xFF, "0xFF"), (0xFFFF, "0xFFFF")]:
        protocol_write("06-Bad-Register",
                       f"6.x  WRITE na nieistniejacy rejestr addr={label}",
                       bad, bytes([0x01]),
                       expected_validation=WRONG_REGISTER)
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 7 – BLEDNY DATA_SIZE
    # =========================================================================
    _section("KATEGORIA 7: Bledny data_size (WRONG_DATA_SIZE)")

    def _write_bad_size(test_label, reg, decl_size):
        log    = CommunicationLog()
        passed = False
        _test_header(test_label, "07-Bad-DataSize",
                     f"declared={decl_size}B  reg={REG_NAMES.get(reg, reg)}")
        try:
            if not do_handshake(log):
                stats.add("07-Bad-DataSize", False)
                return False
            header = build_header(next_frame_id(), OPERATION_WRITE, reg, decl_size)
            do_send_header(header, log)
            val = do_recv_validation(log)
            _print_val(val, WRONG_DATA_SIZE)
            if val == WRONG_DATA_SIZE:
                rprint("    OK: Slave odrzucil bledny data_size")
                time.sleep(DELAY_AFTER_ERROR_VAL)
                rprint("  PASSED\n")
                stats.add("07-Bad-DataSize", True)
                passed = True
                return True
            else:
                stats.add("07-Bad-DataSize", False)
                return False
        except Exception as e:
            rprint(f"    EXCEPTION: {e}")
            stats.add("07-Bad-DataSize", False)
            return False
        finally:
            if not passed:
                log.print_flow()

    _write_bad_size("7.1  data_size=17 (>MAX 16)",                 REG_HOMING, 17)
    time.sleep(INTER_TEST_DELAY)
    _write_bad_size("7.2  data_size=255",                          REG_HOMING, 255)
    time.sleep(INTER_TEST_DELAY)
    _write_bad_size("7.3  REG_HOMING(1B) z data_size=4",          REG_HOMING, 4)
    time.sleep(INTER_TEST_DELAY)
    _write_bad_size("7.4  REG_MOVE_ANGLE(4B) z data_size=8",      REG_MOVE_ANGLE, 8)
    time.sleep(INTER_TEST_DELAY)
    _write_bad_size("7.5  data_size=0 (pusty payload)",            REG_HOMING, 0)
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 8 – DUPLIKAT FRAME_ID
    # =========================================================================
    _section("KATEGORIA 8: Duplikat frame_id (WRONG_FRAME_ID)")

    fid_dup = next_frame_id()
    protocol_write("08-DupFrameID", "8.1  Pierwszy WRITE z frame_id",
                   REG_HOMING, bytes([0x01]), frame_id=fid_dup)
    time.sleep(INTER_TEST_DELAY)
    protocol_write("08-DupFrameID", "8.2  Ten sam frame_id – oczekuj WRONG_FRAME_ID",
                   REG_HOMING, bytes([0x00]), frame_id=fid_dup,
                   expected_validation=WRONG_FRAME_ID)
    time.sleep(INTER_TEST_DELAY)
    protocol_write("08-DupFrameID", "8.3  Nowy frame_id – powinien przejsc",
                   REG_HOMING, bytes([0x00]))
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 9 – BLEDNA OPERACJA
    # =========================================================================
    _section("KATEGORIA 9: Nieprawidlowa operacja (WRONG_OPERATION)")

    # READ z write-only
    protocol_read("09-BadOperation",
                  "9.1  READ z write-only rejestru (REG_DIAG_CONTROL)",
                  REG_DIAG_CONTROL, REG_SIZES[REG_DIAG_CONTROL],
                  expected_validation=WRONG_OPERATION)
    time.sleep(INTER_TEST_DELAY)

    # WRITE do read-only
    protocol_write("09-BadOperation",
                   "9.2  WRITE do read-only rejestru (REG_DIAG_STATUS)",
                   REG_DIAG_STATUS, struct.pack(">I", 0x00),
                   expected_validation=WRONG_OPERATION)
    time.sleep(INTER_TEST_DELAY)

    # Nieprawidlowy operation_type
    def _bad_op_type(test_label, bad_op):
        log    = CommunicationLog()
        passed = False
        _test_header(test_label, "09-BadOperation",
                     f"operation_type=0x{bad_op:02X}")
        try:
            if not do_handshake(log):
                stats.add("09-BadOperation", False)
                return False
            fid = next_frame_id()
            # Ręcznie budujemy header z nieprawidłowym operation_type
            # CRC liczymy z polami z bad_op
            crc_bytes = bytes([
                SLAVE_ID,
                (fid >> 8) & 0xFF, fid & 0xFF,
                bad_op & 0xFF,
                0x00, REG_HOMING & 0xFF,
                0x00, 0x01,
            ])
            crc = crc32_stm32(crc_bytes)
            # format: B B H B H H I B  (8 argumentow)
            header = struct.pack(
                ">BBHBHHIB",
                START_BYTE, SLAVE_ID, fid,
                bad_op & 0xFF,
                REG_HOMING, 1,
                crc, END_BYTE,
            )
            do_send_header(header, log)
            val = do_recv_validation(log)
            _print_val(val, WRONG_OPERATION)
            if val == WRONG_OPERATION:
                rprint("    OK: Slave odrzucil nieprawidlowy operation_type")
                time.sleep(DELAY_AFTER_ERROR_VAL)
                rprint("  PASSED\n")
                stats.add("09-BadOperation", True)
                passed = True
                return True
            else:
                stats.add("09-BadOperation", False)
                return False
        except Exception as e:
            rprint(f"    EXCEPTION: {e}")
            stats.add("09-BadOperation", False)
            return False
        finally:
            if not passed:
                log.print_flow()

    _bad_op_type("9.3  operation_type=0x03", 0x03)
    time.sleep(INTER_TEST_DELAY)
    _bad_op_type("9.4  operation_type=0x00", 0x00)
    time.sleep(INTER_TEST_DELAY)
    _bad_op_type("9.5  operation_type=0xFF", 0xFF)
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 10 – BLEDNE CRC W NAGLOWKU
    # =========================================================================
    _section("KATEGORIA 10: Bledne CRC w naglowku (WRONG_CRC_VALUE)")

    protocol_write("10-Bad-HdrCRC",
                   "10.1 WRITE z celowo blednym CRC naglowka",
                   REG_HOMING, bytes([0x01]),
                   corrupt_header_crc=True,
                   expected_validation=WRONG_CRC_VALUE)
    time.sleep(INTER_TEST_DELAY)

    protocol_read("10-Bad-HdrCRC",
                  "10.2 READ z celowo blednym CRC naglowka",
                  REG_HOMING, REG_SIZES[REG_HOMING],
                  corrupt_header_crc=True,
                  expected_validation=WRONG_CRC_VALUE)
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 11 – BLEDNE CRC W DANYCH (WRITE)
    # =========================================================================
    _section("KATEGORIA 11: Bledne CRC w danych – sciezka WRITE")

    protocol_write("11-Bad-DataCRC",
                   "11.1 WRITE REG_HOMING z blednym CRC danych",
                   REG_HOMING, bytes([0x01]),
                   corrupt_data_crc=True)
    time.sleep(INTER_TEST_DELAY)

    protocol_write("11-Bad-DataCRC",
                   "11.2 WRITE REG_MOVE_ANGLE z blednym CRC danych",
                   REG_MOVE_ANGLE, struct.pack(">I", 180),
                   corrupt_data_crc=True)
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 12 – BLEDNY COMMIT OD MASTERA (READ)
    # =========================================================================
    _section("KATEGORIA 12: Bledny COMMIT od mastera – sciezka READ")

    protocol_read("12-Bad-MasterCommit",
                  "12.1 READ REG_MOVE_ANGLE + bledny COMMIT mastera",
                  REG_MOVE_ANGLE, REG_SIZES[REG_MOVE_ANGLE],
                  send_bad_commit=True)
    time.sleep(INTER_TEST_DELAY)

    protocol_read("12-Bad-MasterCommit",
                  "12.2 READ REG_HOMING + bledny COMMIT mastera",
                  REG_HOMING, REG_SIZES[REG_HOMING],
                  send_bad_commit=True)
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 13 – TESTY TIMEOUTU
    # =========================================================================
    _section("KATEGORIA 13: Testy timeoutu STM32")

    for step in ['syn', 'ack', 'header', 'data']:
        protocol_timeout_test(
            "13-Timeout",
            f"13.x Timeout po kroku: '{step}'",
            abort_after=step,
            delay_s=2.5,
        )
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 14 – READY_SLAVE_FLAG GPIO22
    # =========================================================================
    _section("KATEGORIA 14: READY_SLAVE_FLAG (GPIO22) po wykonaniu callbacku")

    def _test_ready_flag(test_label, reg, payload_bytes, wait_s=1.5):
        log    = CommunicationLog()
        passed = False
        _test_header(test_label, "14-ReadyFlag",
                     f"reg={REG_NAMES.get(reg, reg)}  wait={wait_s}s")
        gpio_before = "LOW" if GPIO.input(READY_PIN) == GPIO.LOW else "HIGH"
        rprint(f"    GPIO22 przed testem: {gpio_before}")
        try:
            ok = protocol_write(
                "14-ReadyFlag", test_label, reg, payload_bytes,
                wait_ready=False, verbose=False,
            )
            if not ok:
                return False
            deadline = time.time() + wait_s
            while time.time() < deadline:
                if GPIO.input(READY_PIN) == GPIO.LOW:
                    elapsed = wait_s - (deadline - time.time())
                    rprint(f"    OK: GPIO22 LOW po ~{elapsed:.3f}s")
                    passed = True
                    stats.add("14-ReadyFlag", True)
                    rprint("  PASSED\n")
                    return True
                time.sleep(0.001)
            rprint(f"    FAIL: GPIO22 nie opadl w ciagu {wait_s}s")
            stats.add("14-ReadyFlag", False)
            return False
        except Exception as e:
            rprint(f"    EXCEPTION: {e}")
            stats.add("14-ReadyFlag", False)
            return False
        finally:
            gpio_after = "LOW" if GPIO.input(READY_PIN) == GPIO.LOW else "HIGH"
            rprint(f"    GPIO22 po tescie: {gpio_after}")

    _test_ready_flag("14.1 READY po WRITE REG_HOMING=1",
                     REG_HOMING, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    _test_ready_flag("14.2 READY po WRITE REG_EMERGENCY_STOP=1",
                     REG_EMERGENCY_STOP, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    _test_ready_flag("14.3 READY po WRITE REG_DIAG_CONTROL=1",
                     REG_DIAG_CONTROL, bytes([0x01]))
    time.sleep(INTER_TEST_DELAY)

    _test_ready_flag("14.4 READY po WRITE REG_MOVE_ANGLE=360",
                     REG_MOVE_ANGLE, struct.pack(">I", 360))
    time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # KAT 15 – STRESS TESTY
    # =========================================================================
    _section("KATEGORIA 15: Stress testy")

    # 15.1 – 10x szybkich WRITE
    rprint("\n--- Test 15.1: 10x szybkich WRITE REG_HOMING ---")
    for i in range(10):
        protocol_write(
            "15-Stress",
            f"15.1.{i+1:02d} WRITE REG_HOMING={i % 2}",
            REG_HOMING, bytes([i % 2]),
            verbose=False,
        )
        time.sleep(0.05)
    time.sleep(INTER_TEST_DELAY)

    # 15.2 – 5x round-trip WRITE+READ
    rprint("\n--- Test 15.2: 5x round-trip WRITE+READ REG_MOVE_ANGLE ---")
    for i in range(5):
        val_w = (i + 1) * 1000
        protocol_write(
            "15-Stress",
            f"15.2.{i+1:02d}W WRITE angle={val_w}",
            REG_MOVE_ANGLE, struct.pack(">I", val_w),
            verbose=False,
        )
        time.sleep(INTER_TEST_DELAY)
        ok, data = protocol_read(
            "15-Stress",
            f"15.2.{i+1:02d}R READ angle",
            REG_MOVE_ANGLE, REG_SIZES[REG_MOVE_ANGLE],
            verbose=False,
        )
        if ok:
            rd    = struct.unpack(">I", data)[0]
            match = "OK" if rd == val_w else "FAIL"
            rprint(f"    Round-trip [{match}]: zapisano={val_w}  odczytano={rd}")
        time.sleep(INTER_TEST_DELAY)

    # 15.3 – Przemienne poprawne/bledne ramki
    rprint("\n--- Test 15.3: Przemienne poprawne/bledne ramki ---")
    for i in range(4):
        protocol_write(
            "15-Stress",
            f"15.3.{i*2+1:02d} Poprawny WRITE",
            REG_HOMING, bytes([0x01]),
            verbose=False,
        )
        time.sleep(INTER_TEST_DELAY)
        protocol_write(
            "15-Stress",
            f"15.3.{i*2+2:02d} Bledny WRITE (corrupt header CRC)",
            REG_HOMING, bytes([0x00]),
            corrupt_header_crc=True,
            expected_validation=WRONG_CRC_VALUE,
            verbose=False,
        )
        time.sleep(INTER_TEST_DELAY)

    # 15.4 – Wszystkie rejestry po kolei (WRITE gdzie mozliwe)
    rprint("\n--- Test 15.4: Wszystkie rejestry R/W ---")
    for reg in [REG_HOMING, REG_MOVE_ANGLE, REG_EMERGENCY_STOP]:
        size    = REG_SIZES[reg]
        payload = bytes([0xA0 + reg] * size)
        protocol_write(
            "15-Stress",
            f"15.4 WRITE {REG_NAMES[reg]}",
            reg, payload, verbose=False,
        )
        time.sleep(INTER_TEST_DELAY)
        protocol_read(
            "15-Stress",
            f"15.4 READ  {REG_NAMES[reg]}",
            reg, size, verbose=False,
        )
        time.sleep(INTER_TEST_DELAY)

    # =========================================================================
    # PODSUMOWANIE
    # =========================================================================
    ts_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    rprint(f"\n  Koniec testow: {ts_end}")
    stats.print_summary()


# =============================================================================
# PUNKT WEJSCIA
# =============================================================================
try:
    run_all_tests()
finally:
    save_report()
    spi.close()
    GPIO.cleanup()
    print("Zasoby zwolnione.")