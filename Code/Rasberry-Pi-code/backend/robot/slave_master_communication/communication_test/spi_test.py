import spidev
import RPi.GPIO as GPIO
import time

# ---------------------------
# KONFIGURACJA GPIO
# ---------------------------
CS_PIN = 5   
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

# ---------------------------
# KONFIGURACJA SPI
# ---------------------------
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500000  # 500kHz
spi.mode = 0b00
spi.no_cs = True

# ---------------------------
# KONSTANTY PROTOKOŁU
# ---------------------------
START_BYTE = 0x55
SLAVE_ID = 0x01
END_BYTE = 0xAA
SYN_BYTE = 0x97
SYN_ACK_BYTE = 0x98
ACK_BYTE = 0x99

# Kody walidacji
VALIDATION_OK = 0x01
WRONG_START_BYTE = 0x02
WRONG_SLAVE_ID = 0x03
WRONG_COMMAND = 0x04
WRONG_DATA_SIZE = 0x05
WRONG_END_BYTE = 0x06

# Komendy
CMD_HOMING = 100
CMD_MOVE_VIA_ANGLE = 101
CMD_DIAGNOSTIC = 102

# Timing
DELAY_BETWEEN_STEPS = 0.005  # 5ms między krokami
CS_SETUP_TIME = 0.0001       # 100μs na ustabilizowanie CS

# ---------------------------
# FUNKCJE POMOCNICZE
# ---------------------------
def send_receive_cs(data):
    """Wysyła dane przez SPI z kontrolą CS."""
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(CS_SETUP_TIME)
    
    resp = spi.xfer2(data)
    
    GPIO.output(CS_PIN, GPIO.HIGH)
    return resp

def build_header(command, data_size, frame_id=0x01):
    """
    Buduje header zgodnie ze strukturą Header_t (7 bajtów):
    start_byte, length, slave_id, frame_id, command, data_size, end_byte
    """
    HEADER_SIZE = 7
    length = HEADER_SIZE + data_size
    
    return [
        START_BYTE,    # start_byte
        length,        # length (całkowita długość ramki)
        SLAVE_ID,      # slave_id
        frame_id,      # frame_id
        command,       # command
        data_size,     # data_size
        END_BYTE       # end_byte
    ]

def get_validation_error_name(code):
    """Zwraca nazwę błędu walidacji."""
    errors = {
        VALIDATION_OK: "OK",
        WRONG_START_BYTE: "WRONG_START_BYTE",
        WRONG_SLAVE_ID: "WRONG_SLAVE_ID",
        WRONG_COMMAND: "WRONG_COMMAND",
        WRONG_DATA_SIZE: "WRONG_DATA_SIZE",
        WRONG_END_BYTE: "WRONG_END_BYTE"
    }
    return errors.get(code, f"UNKNOWN(0x{code:02X})")

# ---------------------------
# GŁÓWNA FUNKCJA PROTOKOŁU
# ---------------------------
def run_full_protocol(command, payload, frame_id=0x01):
    """
    Wykonuje pełną sekwencję protokołu bez przerw.
    Zwraca True jeśli sukces, False jeśli błąd.
    """
    try:
        data_size = len(payload)
        print(f"\n{'='*60}")
        print(f"Start sesji: CMD={command}, Frame_ID=0x{frame_id:02X}, Data={data_size}B")
        print(f"{'='*60}")

        # === HANDSHAKE ===
        # 1. SYN
        print("→ Wysyłam SYN (0x97)")
        send_receive_cs([SYN_BYTE])
        time.sleep(DELAY_BETWEEN_STEPS)

        # 2. SYN_ACK
        print("← Odbieram SYN_ACK")
        resp_syn_ack = send_receive_cs([0xFF])
        if resp_syn_ack[0] == SYN_ACK_BYTE:
            print(f"  ✓ SYN_ACK OK (0x{resp_syn_ack[0]:02X})")
        else:
            print(f"  ✗ SYN_ACK błędny! Otrzymano 0x{resp_syn_ack[0]:02X}")
            return False
        time.sleep(DELAY_BETWEEN_STEPS)

        # 3. ACK
        print("→ Wysyłam ACK (0x99)")
        send_receive_cs([ACK_BYTE])
        time.sleep(DELAY_BETWEEN_STEPS)

        # === RAMKA DANYCH ===
        # 4. HEADER
        header = build_header(command, data_size, frame_id)
        print(f"→ Wysyłam HEADER: {[f'0x{b:02X}' for b in header]}")
        send_receive_cs(header)
        time.sleep(DELAY_BETWEEN_STEPS)

        # 5. VALIDATION CODE
        print("← Odbieram VALIDATION_CODE")
        resp_validation = send_receive_cs([0xFF])
        validation_code = resp_validation[0]
        error_name = get_validation_error_name(validation_code)
        
        if validation_code == VALIDATION_OK:
            print(f"  ✓ Walidacja OK (0x{validation_code:02X})")
        else:
            print(f"  ✗ Walidacja FAILED: {error_name}")
            print("  STM32 przerwał komunikację i wrócił do WAIT_SYN")
            return False
        time.sleep(DELAY_BETWEEN_STEPS)

        # 6. DATA
        print(f"→ Wysyłam DATA ({data_size}B): {[f'0x{b:02X}' for b in payload]}")
        send_receive_cs(payload)
        time.sleep(DELAY_BETWEEN_STEPS)

        # 7. ECHO
        # Echo = start_byte + command + slave_id + frame_id + data + end_byte
        echo_size = 1 + 1 + 1 + 1 + data_size + 1
        print(f"← Odbieram ECHO ({echo_size}B)")
        echo_resp = send_receive_cs([0xFF] * echo_size)
        
        # Parsowanie ECHO
        echo_start = echo_resp[0]
        echo_cmd = echo_resp[1]
        echo_slave = echo_resp[2]
        echo_frame = echo_resp[3]
        echo_data = echo_resp[4:4+data_size]
        echo_end = echo_resp[4+data_size]
        
        print(f"  ECHO: start=0x{echo_start:02X}, cmd={echo_cmd}, "
              f"slave=0x{echo_slave:02X}, frame=0x{echo_frame:02X}")
        print(f"        data={[f'0x{b:02X}' for b in echo_data]}, end=0x{echo_end:02X}")
        
        # Weryfikacja ECHO
        if echo_data == payload:
            print("  ✓ Echo danych poprawne")
        else:
            print("  ✗ Echo danych różni się od wysłanych!")
        
        time.sleep(DELAY_BETWEEN_STEPS)

        # 8. COMMIT
        print("→ Wysyłam COMMIT (0x11)")
        send_receive_cs([0x11])
        
        print(f"{'='*60}")
        print("✓ Sesja zakończona SUKCESEM")
        print(f"{'='*60}\n")
        return True

    except Exception as e:
        print(f"\n✗ BŁĄD w transmisji: {e}\n")
        return False

# ---------------------------
# TESTY
# ---------------------------
def run_tests():
    """Uruchamia serię testów protokołu."""
    
    tests = [
        # Test 1: Poprawna ramka z HOMING
        {
            "name": "Test 1: HOMING command - poprawna ramka",
            "command": CMD_HOMING,
            "payload": [0x10, 0x20, 0x30],
            "frame_id": 0x01
        },
        
        # Test 2: Poprawna ramka z MOVE_VIA_ANGLE
        {
            "name": "Test 2: MOVE_VIA_ANGLE - większy payload",
            "command": CMD_MOVE_VIA_ANGLE,
            "payload": [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
            "frame_id": 0x42
        },
        
        # Test 3: Poprawna ramka z DIAGNOSTIC
        {
            "name": "Test 3: DIAGNOSTIC - mały payload",
            "payload": [0x01, 0x02],
            "command": CMD_DIAGNOSTIC,
            "frame_id": 0xFF
        },
        
        # Test 4: Kolejna sesja - sprawdzenie czy timer się zresetował
        {
            "name": "Test 4: Kolejna sesja po sukcesie",
            "command": CMD_HOMING,
            "payload": [0x99, 0x88, 0x77, 0x66],
            "frame_id": 0x10
        }
    ]
    
    results = []
    
    for i, test in enumerate(tests, 1):
        print(f"\n\n{'#'*60}")
        print(f"# {test['name']}")
        print(f"{'#'*60}")
        
        success = run_full_protocol(
            command=test['command'],
            payload=test['payload'],
            frame_id=test['frame_id']
        )
        
        results.append({
            "test": test['name'],
            "success": success
        })
        
        # Przerwa między sesjami (żeby timer STM32 się zresetował)
        time.sleep(0.2)
    
    # Podsumowanie
    print(f"\n\n{'='*60}")
    print("PODSUMOWANIE TESTÓW")
    print(f"{'='*60}")
    for r in results:
        status = "✓ PASS" if r['success'] else "✗ FAIL"
        print(f"{status} - {r['test']}")
    
    passed = sum(1 for r in results if r['success'])
    print(f"\nWynik: {passed}/{len(results)} testów zaliczonych")
    print(f"{'='*60}\n")

# ---------------------------
# GŁÓWNA PĘTLA PROGRAMU
# ---------------------------
try:
    print("\n" + "="*60)
    print("SPI PROTOCOL TESTER - Automatyczny test z timerem")
    print("="*60)
    
    run_tests()
    
    # Opcjonalnie: test manualny
    # run_full_protocol(CMD_HOMING, [0xDE, 0xAD, 0xBE, 0xEF], frame_id=0x99)

finally:
    spi.close()
    GPIO.cleanup()
    print("Zasoby zwolnione.")