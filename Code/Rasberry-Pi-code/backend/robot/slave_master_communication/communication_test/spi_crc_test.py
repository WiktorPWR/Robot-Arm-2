"""
=============================================================================
  PROGRAM DIAGNOSTYCZNY – ZNAJDOWANIE POPRAWNEGO ALGORYTMU CRC STM32
=============================================================================
  Plik: spi_crc_finder.py

  CEL:
    STM32 uzywa wbudowanego sprzętowego CRC unit (polynomial 0x04C11DB7).
    Sposob wyliczania zalezY od:
      - kolejnosci bajtow w 32-bit wordach (big vs little endian)
      - ustawien REV_IN / REV_OUT w rejestrze CRC->CR
      - wartosci poczatkowej (init=0xFFFFFFFF vs 0x00000000)
      - które dokladnie bajty headera ida do CRC

    Program wyslya 10 headerow, kazdy z CRC policzonym innym algorytmem.
    Sprawdza ktory STM32 odpowie OK_FRAME (0x00) zamiast WRONG_CRC_VALUE (0x07).

  ALGORYTMY CRC (polynomial 0x04C11DB7, jak w STM32 HAL):
    V1  big-endian words,    init=0xFFFFFFFF  (moja obecna impl)
    V2  little-endian words, init=0xFFFFFFFF  (STM32 memcpy LE)
    V3  byte-by-byte MSB,    init=0xFFFFFFFF
    V4  little-endian words, init=0x00000000
    V5  big-endian words,    init=0x00000000
    V6  REV_IN byte-swap,    init=0xFFFFFFFF
    V7  REV_IN bit-reflect,  init=0xFFFFFFFF
    V8  standard CRC32 (zlib)
    V9  LE words bez paddingu (tylko pelne wordy)
    V10 CRC liczony z CALYM headerem (14B) zamiast 8B

  WYMAGANIA GPIO:
    CS_PIN    = 5   (chip select)
    READY_PIN = 22  (READY_SLAVE_FLAG od STM32)

  UZYCIE:
    python3 spi_crc_finder.py
    
  INTERPRETACJA WYNIKU:
    Gdy jakis wariant dostanie OK_FRAME -> ten algorytm jest poprawny.
    Wyniki sa zapisywane do spi_crc_result_YYYYMMDD_HHMMSS.txt
=============================================================================
"""

import spidev
import RPi.GPIO as GPIO
import time
import struct
import zlib
from datetime import datetime

# =============================================================================
# RAPORT
# =============================================================================
_log = []

def pr(*args):
    line = " ".join(str(a) for a in args)
    print(line)
    _log.append(line)

def save_report(filename):
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(_log) + "\n")
    pr(f"\n[RAPORT] Zapisano: {filename}")


# =============================================================================
# KONFIGURACJA
# =============================================================================
CS_PIN    = 5
READY_PIN = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN,    GPIO.OUT);  GPIO.output(CS_PIN, GPIO.HIGH)
GPIO.setup(READY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500_000
spi.mode         = 0b00
spi.no_cs        = True

# Stale protokolu
START_BYTE   = 0x55
SLAVE_ID     = 0x01
END_BYTE     = 0xAA
SYN_BYTE     = 0x97
SYN_ACK_BYTE = 0x98
ACK_BYTE     = 0x99

OPERATION_WRITE = 0x02
REG_HOMING      = 0x0000

OK_FRAME        = 0x00
WRONG_CRC_VALUE = 0x07

# Timming – konserwatywne wartosci
CS_SETUP     = 0.0005   # 500us
STEP_DELAY   = 0.015    # 15ms miedzy krokami
POST_VAL     = 0.050    # 50ms po odebraniu VALIDATION (czas na TxCpltCallback)
TEST_DELAY   = 0.300    # 300ms miedzy testami


# =============================================================================
# NISKA WARSTWA SPI
# =============================================================================

def xfer(data):
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(CS_SETUP)
    resp = spi.xfer2(list(data))
    GPIO.output(CS_PIN, GPIO.HIGH)
    return resp


def handshake():
    """Wykonuje SYN->SYN_ACK->ACK. Ponawia jesli slave zablokowany."""
    for attempt in range(5):
        xfer([SYN_BYTE])
        time.sleep(STEP_DELAY)
        resp = xfer([0xFF])
        time.sleep(STEP_DELAY)

        if resp[0] == SYN_ACK_BYTE:
            xfer([ACK_BYTE])
            time.sleep(STEP_DELAY)
            return True

        pr(f"    [HS] proba {attempt+1}: SYN_ACK=0x{resp[0]:02X}, czekam...")
        time.sleep(0.150)

    pr("    [HS] FAIL: slave nie odpowiada na SYN")
    return False


def send_header_get_validation(header_bytes):
    """
    Wysyla header i odbiera kod walidacji.
    Czeka POST_VAL po odebraniu kodu zeby slave zdazyl wykonac TxCpltCallback.
    Zwraca (validation_code, miso_przy_syn).
    """
    # SYN
    resp_syn = xfer([SYN_BYTE])
    time.sleep(STEP_DELAY)

    # SYN_ACK
    resp_ack = xfer([0xFF])
    time.sleep(STEP_DELAY)

    if resp_ack[0] != SYN_ACK_BYTE:
        return None, resp_syn[0]  # slave zablokowany

    # ACK
    xfer([ACK_BYTE])
    time.sleep(STEP_DELAY)

    # HEADER
    xfer(list(header_bytes))
    time.sleep(STEP_DELAY)

    # VALIDATION CODE
    resp_val = xfer([0xFF])

    # KLUCZOWE: czekaj az slave wykona TxCpltCallback i zresetuje stan
    time.sleep(POST_VAL)

    return resp_val[0], resp_syn[0]


# =============================================================================
# ALGORYTMY CRC
# =============================================================================

def _crc_engine_be(data, init=0xFFFFFFFF, poly=0x04C11DB7):
    """
    Rdzen CRC: big-endian words, MSB-first.
    Data jest paddowana do wielokrotnosci 4 bajtow zerami.
    """
    padded = bytes(data) + b'\x00' * ((4 - len(data) % 4) % 4)
    words  = struct.unpack(f'>{len(padded)//4}I', padded)
    crc = init & 0xFFFFFFFF
    for w in words:
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def _crc_engine_le(data, init=0xFFFFFFFF, poly=0x04C11DB7):
    """
    Rdzen CRC: little-endian words (jak memcpy na STM32 LE).
    Data jest kopiowana do uint32_t[] w pamieci LE -> bajty laduja LE.
    """
    padded = bytes(data) + b'\x00' * ((4 - len(data) % 4) % 4)
    words  = struct.unpack(f'<{len(padded)//4}I', padded)
    crc = init & 0xFFFFFFFF
    for w in words:
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def _crc_engine_le_rev_in(data, init=0xFFFFFFFF, poly=0x04C11DB7):
    """
    LE words + REV_IN=01 (odwrocenie bajtow w wordzie przed przetworzeniem).
    Odpowiada ustawieniu CRC->CR.REV_IN = 01 na STM32.
    """
    padded = bytes(data) + b'\x00' * ((4 - len(data) % 4) % 4)
    words  = struct.unpack(f'<{len(padded)//4}I', padded)
    crc = init & 0xFFFFFFFF
    for w in words:
        # byte-swap: [B3 B2 B1 B0] -> [B0 B1 B2 B3]
        rev = struct.unpack('>I', struct.pack('<I', w))[0]
        crc ^= rev
        for _ in range(32):
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def _crc_engine_be_rev_in_bits(data, init=0xFFFFFFFF, poly=0x04C11DB7):
    """
    BE words + REV_IN=10 (odbicie bitowe kazdego bitu w wordzie).
    """
    def rev_bits(w):
        result = 0
        for _ in range(32):
            result = (result << 1) | (w & 1)
            w >>= 1
        return result

    padded = bytes(data) + b'\x00' * ((4 - len(data) % 4) % 4)
    words  = struct.unpack(f'>{len(padded)//4}I', padded)
    crc = init & 0xFFFFFFFF
    for w in words:
        crc ^= rev_bits(w)
        for _ in range(32):
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def _crc_bytewise(data, init=0xFFFFFFFF, poly=0x04C11DB7):
    """Bajt-po-bajcie, MSB first (bez paddowania do word)."""
    crc = init & 0xFFFFFFFF
    for b in data:
        crc ^= (b << 24)
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFFFFFFFF if (crc & 0x80000000) else (crc << 1) & 0xFFFFFFFF
    return crc


def _rev32(v):
    """Odwroc 32 bity."""
    return int(f'{v:032b}'[::-1], 2)


# =============================================================================
# DEFINICJE WARIANTOW CRC
# =============================================================================

# Kazdy wariant to (nazwa, opis, funkcja(data)->uint32)
CRC_VARIANTS = [
    (
        "V1",
        "BE-words, init=0xFFFFFFFF, poly=0x04C11DB7  [obecna impl]",
        lambda d: _crc_engine_be(d, init=0xFFFFFFFF),
    ),
    (
        "V2",
        "LE-words, init=0xFFFFFFFF, poly=0x04C11DB7  [STM32 memcpy bez REV]",
        lambda d: _crc_engine_le(d, init=0xFFFFFFFF),
    ),
    (
        "V3",
        "LE-words + REV_IN=01 (byte-swap), init=0xFFFFFFFF",
        lambda d: _crc_engine_le_rev_in(d, init=0xFFFFFFFF),
    ),
    (
        "V4",
        "bytewise MSB-first, init=0xFFFFFFFF, poly=0x04C11DB7",
        lambda d: _crc_bytewise(d, init=0xFFFFFFFF),
    ),
    (
        "V5",
        "BE-words, init=0x00000000, poly=0x04C11DB7",
        lambda d: _crc_engine_be(d, init=0x00000000),
    ),
    (
        "V6",
        "LE-words, init=0x00000000, poly=0x04C11DB7",
        lambda d: _crc_engine_le(d, init=0x00000000),
    ),
    (
        "V7",
        "BE-words REV_IN=10 (bit-reflect per word), init=0xFFFFFFFF",
        lambda d: _crc_engine_be_rev_in_bits(d, init=0xFFFFFFFF),
    ),
    (
        "V8",
        "Standard CRC32 (zlib/Ethernet), poly=0xEDB88320 reflected",
        lambda d: zlib.crc32(d) & 0xFFFFFFFF,
    ),
    (
        "V9",
        "LE-words + REV_IN=01, init=0x00000000",
        lambda d: _crc_engine_le_rev_in(d, init=0x00000000),
    ),
    (
        "V10",
        "BE-words, init=0xFFFFFFFF – CRC z CALEGO headera 12B (bez start/end/crc)",
        None,   # specjalny – patrz nizej
    ),
]


# =============================================================================
# BUDOWANIE HEADERA
# =============================================================================

def build_header_with_crc(frame_id, crc_value,
                           start=START_BYTE, slave=SLAVE_ID, end=END_BYTE,
                           op=OPERATION_WRITE, addr=REG_HOMING, size=1):
    """Buduje 14-bajtowy header z podanym CRC (juz obliczonym)."""
    return struct.pack(
        ">BBHBHHIB",
        start & 0xFF,
        slave & 0xFF,
        frame_id & 0xFFFF,
        op & 0xFF,
        addr & 0xFFFF,
        size & 0xFFFF,
        crc_value & 0xFFFFFFFF,
        end & 0xFF,
    )


def get_crc_input_bytes(frame_id, op=OPERATION_WRITE, addr=REG_HOMING, size=1):
    """
    Zwraca 8 bajtow ktore ida do CRC w STM32:
    slave_id(1) + frame_id(2) + op(1) + addr(2) + size(2)
    Zgodnie z: calculate_crc(&rx_header.slave_id, HEADER_SIZE_WITHOUT_CRC)
    gdzie HEADER_SIZE_WITHOUT_CRC = 14 - 4 - 2 = 8
    """
    return bytes([
        SLAVE_ID,
        (frame_id >> 8) & 0xFF, frame_id & 0xFF,
        op & 0xFF,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (size >> 8) & 0xFF, size & 0xFF,
    ])


def get_crc_input_full_header(frame_id, op=OPERATION_WRITE, addr=REG_HOMING, size=1):
    """
    Wariant V10: CRC z calego headera BEZ start_byte, end_byte i pola CRC.
    = slave_id(1) + frame_id(2) + op(1) + addr(2) + size(2) = 8B
    ... to samo co standard, ale sprawdzamy czy moze jest wlaczone wiecej pol
    Tutaj probujemy 12B: slave_id + frame_id + op + addr + size + (4B zer w miejscu CRC)
    """
    return bytes([
        SLAVE_ID,
        (frame_id >> 8) & 0xFF, frame_id & 0xFF,
        op & 0xFF,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (size >> 8) & 0xFF, size & 0xFF,
        0x00, 0x00, 0x00, 0x00,  # pole CRC jako zera
    ])


# =============================================================================
# GLOWNA FUNKCJA DIAGNOSTYCZNA
# =============================================================================

def run_crc_finder():
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    report_file = f"spi_crc_result_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

    pr("=" * 70)
    pr(f"  DIAGNOSTYKA CRC STM32  |  {ts}")
    pr("=" * 70)
    pr()
    pr("Cel: znalezc ktory algorytm CRC STM32 akceptuje (odpowiedz OK_FRAME=0x00)")
    pr("Kazdy test wysyla poprawny header z innym CRC i sprawdza odpowiedz.")
    pr()
    pr("Dane do CRC (8B): slave_id + frame_id[2] + op + addr[2] + size[2]")
    pr("Zgodnie z: calculate_crc(&rx_header.slave_id, HEADER_SIZE_WITHOUT_CRC)")
    pr()

    # Oblicz i wyswietl wartosci CRC dla frame_id=0x0001 (referencyjne)
    ref_fid  = 0x0001
    ref_data = get_crc_input_bytes(ref_fid)
    pr(f"{'─'*70}")
    pr(f"Dane referencyjne (fid=0x{ref_fid:04X}): {[f'0x{b:02X}' for b in ref_data]}")
    pr(f"{'─'*70}")
    for name, desc, fn in CRC_VARIANTS:
        if fn is None:
            data_v10 = get_crc_input_full_header(ref_fid)
            crc_val  = _crc_engine_be(data_v10)
        else:
            crc_val  = fn(ref_data)
        pr(f"  {name:4s}  0x{crc_val:08X}  {desc}")
    pr()

    # =========================================================================
    # FAZA 1: Test wszystkich wariantow CRC, frame_id rosnace
    # =========================================================================
    pr("=" * 70)
    pr("  FAZA 1: Test wszystkich wariantow CRC (jeden header na wariant)")
    pr("=" * 70)
    pr()

    results = []
    frame_id = 0x0010  # startujemy od 0x10 zeby uniknac kolizji z poprzednimi sesjami

    for i, (name, desc, fn) in enumerate(CRC_VARIANTS):
        fid       = frame_id + i
        crc_input = get_crc_input_bytes(fid)

        # Oblicz CRC tym wariantem
        if fn is None:
            data_for_crc = get_crc_input_full_header(fid)
            crc_val = _crc_engine_be(data_for_crc)
        else:
            crc_val = fn(crc_input)

        header = build_header_with_crc(fid, crc_val)

        pr(f"{'─'*70}")
        pr(f"TEST {name}: {desc}")
        pr(f"  fid=0x{fid:04X}  dane_crc={[f'0x{b:02X}' for b in crc_input]}")
        pr(f"  CRC=0x{crc_val:08X}  header={[f'0x{b:02X}' for b in header]}")

        val_code, miso_syn = send_header_get_validation(header)

        if val_code is None:
            pr(f"  WYNIK: BLAD HANDSHAKE (slave zablokowany, MISO_SYN=0x{miso_syn:02X})")
            results.append((name, desc, crc_val, "HANDSHAKE_FAIL"))
        else:
            status = "*** OK_FRAME – CRC POPRAWNE! ***" if val_code == OK_FRAME else \
                     f"WRONG_CRC (0x{val_code:02X})" if val_code == WRONG_CRC_VALUE else \
                     f"OTHER_ERROR (0x{val_code:02X})"
            pr(f"  WYNIK: {status}")
            results.append((name, desc, crc_val, val_code))

        time.sleep(TEST_DELAY)

    # =========================================================================
    # FAZA 2: Jesli zaden nie trafil – test dodatkowych wariantow
    # =========================================================================
    any_ok = any(r[3] == OK_FRAME for r in results)

    if not any_ok:
        pr()
        pr("=" * 70)
        pr("  FAZA 2: Zadny wariant nie trafil – testuje dodatkowe hipotezy")
        pr("=" * 70)
        pr()

        frame_id2 = 0x0020

        extra_variants = [
            (
                "E1",
                "LE-words, init=0xFFFFFFFF – ale dane: caly header 14B bez CRC pol",
                lambda d: _crc_engine_le(d),
                lambda fid: bytes([
                    START_BYTE, SLAVE_ID,
                    (fid >> 8) & 0xFF, fid & 0xFF,
                    OPERATION_WRITE,
                    0x00, REG_HOMING & 0xFF,
                    0x00, 0x01,
                ]),  # 9 bajtow: start + slave + fid + op + addr + size
            ),
            (
                "E2",
                "BE-words, init=0xFFFFFFFF – dane: start_byte tez wlaczony (9B)",
                lambda d: _crc_engine_be(d),
                lambda fid: bytes([
                    START_BYTE, SLAVE_ID,
                    (fid >> 8) & 0xFF, fid & 0xFF,
                    OPERATION_WRITE,
                    0x00, REG_HOMING & 0xFF,
                    0x00, 0x01,
                ]),
            ),
            (
                "E3",
                "LE-words + REV_IN=01, init=0xFFFF – dane: 8B standardowe",
                lambda d: _crc_engine_le_rev_in(d),
                lambda fid: get_crc_input_bytes(fid),
            ),
            (
                "E4",
                "LE-words, init=0xFFFF – dane: 9B (od slave_id do size, 1B paddingu)",
                lambda d: _crc_engine_le(d),
                lambda fid: get_crc_input_bytes(fid) + b'\x00',  # + 1B pad
            ),
            (
                "E5",
                "BE-words, init=0xFFFF – dane: tylko 4B (slave_id+fid[2]+op)",
                lambda d: _crc_engine_be(d),
                lambda fid: bytes([SLAVE_ID,
                                   (fid >> 8) & 0xFF, fid & 0xFF,
                                   OPERATION_WRITE]),
            ),
            (
                "E6",
                "LE-words, init=0xFFFF – dane: tylko 4B (slave_id+fid[2]+op)",
                lambda d: _crc_engine_le(d),
                lambda fid: bytes([SLAVE_ID,
                                   (fid >> 8) & 0xFF, fid & 0xFF,
                                   OPERATION_WRITE]),
            ),
        ]

        for i, (name, desc, fn, data_fn) in enumerate(extra_variants):
            fid       = frame_id2 + i
            crc_input = data_fn(fid)
            crc_val   = fn(crc_input)
            header    = build_header_with_crc(fid, crc_val)

            pr(f"{'─'*70}")
            pr(f"TEST {name}: {desc}")
            pr(f"  fid=0x{fid:04X}  dane({len(crc_input)}B)={[f'0x{b:02X}' for b in crc_input]}")
            pr(f"  CRC=0x{crc_val:08X}")

            val_code, miso_syn = send_header_get_validation(header)

            if val_code is None:
                pr(f"  WYNIK: BLAD HANDSHAKE")
                results.append((name, desc, crc_val, "HANDSHAKE_FAIL"))
            else:
                status = "*** OK_FRAME – CRC POPRAWNE! ***" if val_code == OK_FRAME else \
                         f"WRONG_CRC (0x{val_code:02X})" if val_code == WRONG_CRC_VALUE else \
                         f"OTHER_ERROR (0x{val_code:02X})"
                pr(f"  WYNIK: {status}")
                results.append((name, desc, crc_val, val_code))

            time.sleep(TEST_DELAY)

    # =========================================================================
    # PODSUMOWANIE
    # =========================================================================
    pr()
    pr("=" * 70)
    pr("  PODSUMOWANIE WYNIKOW")
    pr("=" * 70)
    pr()
    pr(f"  {'Wariant':<6} {'CRC':>10}  {'Wynik':<35}  Opis")
    pr(f"  {'─'*6}  {'─'*10}  {'─'*35}  {'─'*25}")

    winners = []
    for name, desc, crc_val, result in results:
        if result == OK_FRAME:
            marker = ">>> OK <<<"
            winners.append((name, desc, crc_val))
        elif result == "HANDSHAKE_FAIL":
            marker = "HS_FAIL"
        elif result == WRONG_CRC_VALUE:
            marker = "WRONG_CRC"
        else:
            marker = f"ERR=0x{result:02X}" if isinstance(result, int) else str(result)
        pr(f"  {name:<6}  0x{crc_val:08X}  {marker:<35}  {desc[:40]}")

    pr()
    if winners:
        pr("╔" + "═" * 68 + "╗")
        pr("║  ZNALEZIONO POPRAWNY ALGORYTM CRC!" + " " * 33 + "║")
        pr("╚" + "═" * 68 + "╝")
        for name, desc, crc_val in winners:
            pr(f"  Wariant: {name}")
            pr(f"  Opis:    {desc}")
            pr(f"  Przykladowe CRC (fid=0x0001): 0x{crc_val:08X}")
    else:
        pr("┌" + "─" * 68 + "┐")
        pr("│  ZADEN WARIANT NIE TRAFIL." + " " * 42 + "│")
        pr("│  Sprawdz logi – slave moze byc w zlym stanie." + " " * 22 + "│")
        pr("│  Mozliwe przyczyny:" + " " * 48 + "│")
        pr("│    1. Slave zablokowany – zrestartuj STM32 i uruchom ponownie." + " " * 5 + "│")
        pr("│    2. CRC liczy wiecej/mniej bajtow niz 8." + " " * 24 + "│")
        pr("│    3. Inne ustawienie REV_IN/REV_OUT w CRC->CR." + " " * 19 + "│")
        pr("│    4. CRC liczy sie z header WLACZNIE z start_byte/end_byte." + " " * 6 + "│")
        pr("└" + "─" * 68 + "┘")

    save_report(report_file)
    return winners


# =============================================================================
# PUNKT WEJSCIA
# =============================================================================
try:
    winners = run_crc_finder()
finally:
    spi.close()
    GPIO.cleanup()
    pr("\nZasoby zwolnione.")