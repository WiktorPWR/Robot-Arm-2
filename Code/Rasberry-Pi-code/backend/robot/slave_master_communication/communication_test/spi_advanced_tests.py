import spidev
import RPi.GPIO as GPIO
import time
from enum import Enum

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
spi.max_speed_hz = 500000
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

# Komendy (100-102)
CMD_HOMING = 100
CMD_MOVE_VIA_ANGLE = 101
CMD_DIAGNOSTIC = 102

# Limity
MAX_DATA_SIZE = 16
HEADER_SIZE = 7

# Timing
DELAY_BETWEEN_STEPS = 0.005
CS_SETUP_TIME = 0.0001
INTER_TEST_DELAY = 0.15

# ---------------------------
# KLASA DO LOGOWANIA KOMUNIKACJI
# ---------------------------
class CommunicationLog:
    """Przechowuje historię komunikacji dla danego testu."""
    def __init__(self):
        self.transactions = []
    
    def add(self, step_name, mosi_data, miso_data):
        """Dodaje transakcję do logu."""
        self.transactions.append({
            'step': step_name,
            'mosi': list(mosi_data),
            'miso': list(miso_data)
        })
    
    def print_flow(self):
        """Wyświetla pełny flow komunikacji."""
        print("\n" + "╔" + "═"*68 + "╗")
        print("║" + " "*20 + "🔍 FLOW KOMUNIKACJI" + " "*29 + "║")
        print("╚" + "═"*68 + "╝")
        
        for i, trans in enumerate(self.transactions, 1):
            print(f"\n{'─'*70}")
            print(f"Krok {i}: {trans['step']}")
            print(f"{'─'*70}")
            print(f"MOSI (Master→Slave): {[f'0x{b:02X}' for b in trans['mosi']]}")
            print(f"MISO (Slave→Master): {[f'0x{b:02X}' for b in trans['miso']]}")
        
        print("\n" + "═"*70 + "\n")

# ---------------------------
# STATYSTYKI
# ---------------------------
class TestStats:
    def __init__(self):
        self.total = 0
        self.passed = 0
        self.failed = 0
        self.categories = {}
    
    def add_result(self, category, passed):
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
        print("\n" + "="*70)
        print("📊 PODSUMOWANIE TESTÓW")
        print("="*70)
        print(f"Łącznie wykonanych testów: {self.total}")
        print(f"✅ Udanych:                {self.passed}")
        print(f"❌ Nieudanych:             {self.failed}")
        
        if self.total > 0:
            success_rate = (self.passed / self.total * 100)
            print(f"📈 Wskaźnik sukcesu:       {success_rate:.1f}%")
        
        print("\n" + "-"*70)
        print("Wyniki według kategorii:")
        print("-"*70)
        
        for category, results in sorted(self.categories.items()):
            total_cat = results["passed"] + results["failed"]
            rate = (results["passed"] / total_cat * 100) if total_cat > 0 else 0
            print(f"{category:30s} {results['passed']:3d}/{total_cat:3d} ({rate:5.1f}%)")
        
        print("="*70)
        
        if self.failed == 0:
            print("🎉 WSZYSTKIE TESTY PRZESZŁY POMYŚLNIE!")
        else:
            print(f"⚠️  {self.failed} testów wymaga uwagi")
        print("="*70 + "\n")

stats = TestStats()

# ---------------------------
# FUNKCJE POMOCNICZE
# ---------------------------
def send_receive_cs(data, comm_log=None, step_name=""):
    """Wysyła dane przez SPI z kontrolą CS i opcjonalnym logowaniem."""
    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(CS_SETUP_TIME)
    resp = spi.xfer2(data)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    # Jeśli podano comm_log, zapisz transakcję
    if comm_log is not None:
        comm_log.add(step_name, data, resp)
    
    return resp

def build_header(command, data_size, frame_id=0x01, 
                 start_byte=START_BYTE, slave_id=SLAVE_ID, end_byte=END_BYTE):
    """Buduje header - z opcją nadpisania wartości (do testów błędów)."""
    length = HEADER_SIZE + data_size
    return [
        start_byte,
        length,
        slave_id,
        frame_id,
        command,
        data_size,
        end_byte
    ]

def get_validation_name(code):
    """Zwraca nazwę kodu walidacji."""
    names = {
        VALIDATION_OK: "OK",
        WRONG_START_BYTE: "WRONG_START_BYTE",
        WRONG_SLAVE_ID: "WRONG_SLAVE_ID",
        WRONG_COMMAND: "WRONG_COMMAND",
        WRONG_DATA_SIZE: "WRONG_DATA_SIZE",
        WRONG_END_BYTE: "WRONG_END_BYTE"
    }
    return names.get(code, f"UNKNOWN(0x{code:02X})")


def reset_spi_peripheral():
    """Agresywny reset sterownika i czyszczenie buforów sprzętowych."""
    global spi
    print("\n🔄 [RESET] Czyszczenie magistrali SPI...")
    
    # 1. Całkowite zamknięcie i usunięcie obiektu
    try:
        spi.close()
    except:
        pass
    
    # 2. Pauza dla systemu operacyjnego (bardzo ważne przy timeoutach!)
    time.sleep(0.1) 
    
    # 3. Ponowna inicjalizacja od zera
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500000
    spi.mode = 0b00
    spi.no_cs = True
    
    # 4. 'Puste' transfery przy podniesionym CS (Slave ignoruje, Master czyści rejestry)
    # To 'wypompowuje' resztki danych z kolejki DMA w Raspberry Pi
    try:
        for _ in range(3):
            spi.xfer2([0x00] * 16)
    except:
        pass
    
    print("✅ [RESET] Sterownik gotowy do pracy.\n")

# ---------------------------
# FUNKCJA TESTOWA - POPRAWNY PROTOKÓŁ
# ---------------------------
def test_valid_protocol(test_name, command, payload, frame_id=0x01, 
                        expected_success=True, category="Valid Protocol",
                        verify_echo_data=False):
    """
    Testuje poprawny przebieg protokołu.
    Zwraca True jeśli test przeszedł zgodnie z oczekiwaniami.
    
    verify_echo_data: jeśli True, weryfikuje że dane w ECHO są identyczne z wysłanymi
                      jeśli False, akceptuje dowolne dane w ECHO
    """
    # Log komunikacji - zapiszemy wszystkie transakcje
    comm_log = CommunicationLog()
    test_passed = False
    
    try:
        # Tworzymy kopię payload
        payload_copy = list(payload)
        data_size = len(payload_copy)
        
        print(f"\n{'─'*70}")
        print(f"🧪 {test_name}")
        print(f"   Kategoria: {category}")
        print(f"   CMD={command}, Frame=0x{frame_id:02X}, Data={data_size}B")
        print(f"{'─'*70}")
        
        # 1. SYN
        print("→ SYN")
        resp = send_receive_cs([SYN_BYTE], comm_log, "1. SYN")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 2. SYN_ACK
        print("← SYN_ACK")
        resp = send_receive_cs([0xFF], comm_log, "2. SYN_ACK (odbieranie)")
        if resp[0] != SYN_ACK_BYTE:
            print(f"  ❌ Błędny SYN_ACK: 0x{resp[0]:02X}")
            stats.add_result(category, False)
            return False
        print(f"  ✓ SYN_ACK OK (0x{resp[0]:02X})")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 3. ACK
        print("→ ACK")
        send_receive_cs([ACK_BYTE], comm_log, "3. ACK")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 4. HEADER
        header = build_header(command, data_size, frame_id)
        print(f"→ HEADER: {[f'0x{b:02X}' for b in header]}")
        send_receive_cs(header, comm_log, "4. HEADER")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 5. VALIDATION CODE
        print("← VALIDATION_CODE")
        resp = send_receive_cs([0xFF], comm_log, "5. VALIDATION_CODE (odbieranie)")
        validation = resp[0]
        val_name = get_validation_name(validation)
        
        if validation == VALIDATION_OK:
            print(f"  ✓ Walidacja: {val_name}")
            
            # 6. DATA
            print(f"→ DATA: {[f'0x{b:02X}' for b in payload_copy]}")
            send_receive_cs(payload_copy, comm_log, "6. DATA")
            time.sleep(DELAY_BETWEEN_STEPS)
            
            # 7. ECHO
            echo_size = 1 + 1 + 1 + 1 + data_size + 1
            print(f"← ECHO ({echo_size}B)")
            
            echo_resp = send_receive_cs([0x00] * echo_size, comm_log, "7. ECHO (odbieranie)")
            
            # Parsowanie ECHO
            echo_start = echo_resp[0]
            echo_cmd = echo_resp[1]
            echo_slave = echo_resp[2]
            echo_frame = echo_resp[3]
            echo_data = list(echo_resp[4:4+data_size])
            echo_end = echo_resp[4+data_size] if (4+data_size) < len(echo_resp) else 0x00
            
            print(f"  ECHO struktura:")
            print(f"    start_byte = 0x{echo_start:02X} (oczekiwano 0x{START_BYTE:02X})")
            print(f"    command    = {echo_cmd} (oczekiwano {command})")
            print(f"    slave_id   = 0x{echo_slave:02X} (oczekiwano 0x{SLAVE_ID:02X})")
            print(f"    frame_id   = 0x{echo_frame:02X} (oczekiwano 0x{frame_id:02X})")
            print(f"    data       = {[f'0x{b:02X}' for b in echo_data]}")
            print(f"    end_byte   = 0x{echo_end:02X} (oczekiwano 0x{END_BYTE:02X})")
            
            # Weryfikacja ECHO
            errors = []
            
            if echo_start != START_BYTE:
                errors.append(f"start_byte: otrzymano 0x{echo_start:02X}, oczekiwano 0x{START_BYTE:02X}")
            
            if echo_cmd != command:
                errors.append(f"command: otrzymano {echo_cmd}, oczekiwano {command}")
            
            if echo_slave != SLAVE_ID:
                errors.append(f"slave_id: otrzymano 0x{echo_slave:02X}, oczekiwano 0x{SLAVE_ID:02X}")
            
            if echo_frame != frame_id:
                errors.append(f"frame_id: otrzymano 0x{echo_frame:02X}, oczekiwano 0x{frame_id:02X}")
            
            # Weryfikuj dane tylko jeśli włączone
            if verify_echo_data:
                if echo_data != payload_copy:
                    errors.append(f"data: otrzymano {[f'0x{b:02X}' for b in echo_data]}, "
                                f"oczekiwano {[f'0x{b:02X}' for b in payload_copy]}")
            else:
                # Nie weryfikujemy zawartości danych - tylko informujemy
                if echo_data != payload_copy:
                    print(f"  ℹ️  Dane w ECHO różnią się (to normalne - zależy od bufora STM32):")
                    print(f"      Wysłano:  {[f'0x{b:02X}' for b in payload_copy]}")
                    print(f"      W ECHO:   {[f'0x{b:02X}' for b in echo_data]}")
                else:
                    print(f"  ✓ Dane w ECHO identyczne z wysłanymi")
            
            if echo_end != END_BYTE:
                errors.append(f"end_byte: otrzymano 0x{echo_end:02X}, oczekiwano 0x{END_BYTE:02X}")
            
            if errors:
                print(f"  ❌ ECHO zawiera błędy:")
                for error in errors:
                    print(f"     - {error}")
                stats.add_result(category, False)
                return False
            else:
                if verify_echo_data:
                    print(f"  ✓ ECHO w pełni poprawny (włącznie z danymi)")
                else:
                    print(f"  ✓ ECHO poprawny (struktura OK, dane nieweryfikowane)")
            
            time.sleep(DELAY_BETWEEN_STEPS)
            
            # 8. COMMIT
            print("→ COMMIT")
            send_receive_cs([0x11], comm_log, "8. COMMIT")
            
            print("✅ Test PASSED")
            test_passed = True
            stats.add_result(category, expected_success)
            return True
            
        else:
            print(f"  ❌ Walidacja: {val_name}")
            print("  Komunikacja przerwana przez STM32")
            
            # Oczekiwaliśmy błędu?
            if not expected_success:
                print("✅ Test PASSED (oczekiwano błędu walidacji)")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print("❌ Test FAILED (nieoczekiwany błąd walidacji)")
                stats.add_result(category, False)
                return False
    
    except Exception as e:
        print(f"❌ Wyjątek: {e}")
        import traceback
        traceback.print_exc()
        stats.add_result(category, False)
        return False
    
    finally:
        # KLUCZOWE: Jeśli test się nie powiódł, pokaż flow
        if not test_passed:
            comm_log.print_flow()

# ---------------------------
# FUNKCJA TESTOWA - BŁĘDNY HEADER
# ---------------------------
def test_invalid_header(test_name, command, payload, frame_id=0x01,
                       start_byte=START_BYTE, slave_id=SLAVE_ID, end_byte=END_BYTE,
                       expected_error=None, category="Header Validation"):
    """
    Testuje reakcję na błędny header.
    expected_error - oczekiwany kod błędu walidacji
    """
    # Log komunikacji
    comm_log = CommunicationLog()
    test_passed = False
    
    try:
        data_size = len(payload)
        print(f"\n{'─'*70}")
        print(f"🧪 {test_name}")
        print(f"   Kategoria: {category}")
        print(f"{'─'*70}")
        
        # 1-3. Handshake
        print("→ SYN")
        send_receive_cs([SYN_BYTE], comm_log, "1. SYN")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        print("← SYN_ACK")
        send_receive_cs([0xFF], comm_log, "2. SYN_ACK (odbieranie)")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        print("→ ACK")
        send_receive_cs([ACK_BYTE], comm_log, "3. ACK")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 4. BŁĘDNY HEADER
        header = build_header(command, data_size, frame_id, start_byte, slave_id, end_byte)
        print(f"→ HEADER (błędny): {[f'0x{b:02X}' for b in header]}")
        send_receive_cs(header, comm_log, "4. HEADER (błędny)")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 5. VALIDATION CODE
        print("← VALIDATION_CODE")
        resp = send_receive_cs([0xFF], comm_log, "5. VALIDATION_CODE (odbieranie)")
        validation = resp[0]
        val_name = get_validation_name(validation)
        
        print(f"  Otrzymano: {val_name} (0x{validation:02X})")
        
        if expected_error is not None:
            if validation == expected_error:
                print(f"  ✓ Otrzymano oczekiwany błąd: {val_name}")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                expected_name = get_validation_name(expected_error)
                print(f"  ❌ Oczekiwano: {expected_name} (0x{expected_error:02X})")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
        else:
            # Jeśli nie określono oczekiwanego błędu, to każdy błąd jest OK
            if validation != VALIDATION_OK:
                print(f"  ✓ STM32 odrzucił błędny header")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print(f"  ❌ STM32 zaakceptował błędny header!")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
    
    except Exception as e:
        print(f"❌ Wyjątek: {e}")
        import traceback
        traceback.print_exc()
        stats.add_result(category, False)
        return False
    
    finally:
        # Jeśli test się nie powiódł, pokaż flow
        if not test_passed:
            comm_log.print_flow()

# ---------------------------
# FUNKCJA TESTOWA - TIMEOUT
# ---------------------------
def test_timeout(test_name, delay_after_step, category="Timeout Tests"):
    """
    Testuje timeout - opóźnienie po danym kroku.
    delay_after_step: 'syn', 'ack', 'header', 'data'
    """
    # Log komunikacji
    comm_log = CommunicationLog()
    test_passed = False
    
    try:
        print(f"\n{'─'*70}")
        print(f"🧪 {test_name}")
        print(f"   Kategoria: {category}")
        print(f"   Opóźnienie po kroku: {delay_after_step}")
        print(f"{'─'*70}")
        
        # 1. SYN
        print("→ SYN")
        send_receive_cs([SYN_BYTE], comm_log, "1. SYN (pierwsza próba)")
        if delay_after_step == 'syn':
            print("  ⏱️  Czekam 2s (timeout)...")
            time.sleep(2.0)
            reset_spi_peripheral()
            print("  Sprawdzam czy STM32 się zresetował...")
            # Próba ponownego SYN
            print("→ SYN (ponownie)")
            resp = send_receive_cs([SYN_BYTE], comm_log, "1b. SYN (po timeout)")
            time.sleep(DELAY_BETWEEN_STEPS)
            print("← SYN_ACK (próba odbioru)")
            resp = send_receive_cs([0xFF], comm_log, "2. SYN_ACK (po timeout)")
            if resp[0] == SYN_ACK_BYTE:
                print("  ✓ STM32 zresetował się i odpowiedział")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print("  ❌ STM32 nie zresetował się poprawnie")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
        
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 2. SYN_ACK
        print("← SYN_ACK")
        send_receive_cs([0xFF], comm_log, "2. SYN_ACK (odbieranie)")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 3. ACK
        print("→ ACK")
        send_receive_cs([ACK_BYTE], comm_log, "3. ACK")
        if delay_after_step == 'ack':
            print("  ⏱️  Czekam 2s (timeout)...")
            time.sleep(2.0)
            print("  Sprawdzam reset...")
            reset_spi_peripheral()
            # Próba nowego handshake
            print("→ SYN (nowy)")
            send_receive_cs([SYN_BYTE], comm_log, "1. SYN (nowy po timeout)")
            time.sleep(DELAY_BETWEEN_STEPS)
            resp = send_receive_cs([0xFF], comm_log, "2. SYN_ACK (nowy po timeout)")
            if resp[0] == SYN_ACK_BYTE:
                print("  ✓ STM32 zresetował się")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print("  ❌ Reset nie zadziałał")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
        
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 4. HEADER
        header = build_header(CMD_HOMING, 3, 0x01)
        print("→ HEADER")
        send_receive_cs(header, comm_log, "4. HEADER")
        if delay_after_step == 'header':
            print("  ⏱️  Czekam 2s (timeout)...")
            time.sleep(2.0)
            print("  Sprawdzam reset...")
            print("→ SYN (nowy)")
            reset_spi_peripheral()
            send_receive_cs([SYN_BYTE], comm_log, "1. SYN (nowy po timeout)")
            time.sleep(DELAY_BETWEEN_STEPS)
            resp = send_receive_cs([0xFF], comm_log, "2. SYN_ACK (nowy po timeout)")
            if resp[0] == SYN_ACK_BYTE:
                print("  ✓ STM32 zresetował się")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print("  ❌ Reset nie zadziałał")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
        
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 5. VALIDATION
        print("← VALIDATION_CODE")
        send_receive_cs([0xFF], comm_log, "5. VALIDATION_CODE (odbieranie)")
        time.sleep(DELAY_BETWEEN_STEPS)
        
        # 6. DATA
        print("→ DATA")
        send_receive_cs([0x10, 0x20, 0x30], comm_log, "6. DATA")
        if delay_after_step == 'data':
            print("  ⏱️  Czekam 2s (timeout)...")
            time.sleep(2.0)
            print("  Sprawdzam reset...")
            print("→ SYN (nowy)")
            reset_spi_peripheral()
            send_receive_cs([SYN_BYTE], comm_log, "1. SYN (nowy po timeout)")
            time.sleep(DELAY_BETWEEN_STEPS)
            resp = send_receive_cs([0xFF], comm_log, "2. SYN_ACK (nowy po timeout)")
            if resp[0] == SYN_ACK_BYTE:
                print("  ✓ STM32 zresetował się")
                print("✅ Test PASSED")
                test_passed = True
                stats.add_result(category, True)
                return True
            else:
                print("  ❌ Reset nie zadziałał")
                print("❌ Test FAILED")
                stats.add_result(category, False)
                return False
        
        print("❌ Test nie powinien dojść do tego punktu")
        stats.add_result(category, False)
        return False
        
    except Exception as e:
        print(f"❌ Wyjątek: {e}")
        reset_spi_peripheral()
        import traceback
        traceback.print_exc()
        stats.add_result(category, False)
        return False
    
    finally:
        # Jeśli test się nie powiódł, pokaż flow
        if not test_passed:
            comm_log.print_flow()
            reset_spi_peripheral()


# ---------------------------
# BATERIA TESTÓW
# ---------------------------
def run_all_tests():
    """Uruchamia wszystkie testy protokołu."""
    
    print("\n" + "="*70)
    print("🚀 KOMPLEKSOWE TESTY PROTOKOŁU SPI")
    print("="*70)
    
    # ========================================
    # KATEGORIA 1: POPRAWNE RAMKI
    # ========================================
    print("\n" + "═"*70)
    print("📦 KATEGORIA 1: POPRAWNE RAMKI DANYCH")
    print("═"*70)
    
    # Test 1.1: Minimalna ramka
    test_valid_protocol(
        "Test 1.1: Minimalna ramka (1 bajt)",
        CMD_HOMING, [0x10], 0x01,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.2: Średnia ramka
    test_valid_protocol(
        "Test 1.2: Średnia ramka (8 bajtów)",
        CMD_MOVE_VIA_ANGLE, [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88], 0x42,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.3: Maksymalna ramka
    test_valid_protocol(
        "Test 1.3: Maksymalna ramka (16 bajtów - MAX_DATA_SIZE)",
        CMD_DIAGNOSTIC, list(range(0x10, 0x20)), 0xFF,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.4: Wszystkie zera
    test_valid_protocol(
        "Test 1.4: Dane same zera",
        CMD_HOMING, [0x00, 0x00, 0x00, 0x00], 0x01,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.5: Wszystkie jedynki
    test_valid_protocol(
        "Test 1.5: Dane same jedynki (0xFF)",
        CMD_HOMING, [0xFF, 0xFF, 0xFF, 0xFF], 0x02,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.6: Wzór przemienny
    test_valid_protocol(
        "Test 1.6: Wzór przemienny (0xAA, 0x55)",
        CMD_MOVE_VIA_ANGLE, [0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55], 0x03,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.7: Bajty specjalne protokołu jako dane
    test_valid_protocol(
        "Test 1.7: Bajty specjalne protokołu w danych",
        CMD_DIAGNOSTIC, [START_BYTE, END_BYTE, SYN_BYTE, SYN_ACK_BYTE, ACK_BYTE], 0x04,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.8: Sekwencja inkrementalna
    test_valid_protocol(
        "Test 1.8: Sekwencja inkrementalna",
        CMD_HOMING, list(range(16)), 0x05,
        category="Valid Protocol"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # Test 1.9: Wszystkie komendy
    for cmd_id, cmd_name in [(CMD_HOMING, "HOMING"), 
                              (CMD_MOVE_VIA_ANGLE, "MOVE_VIA_ANGLE"), 
                              (CMD_DIAGNOSTIC, "DIAGNOSTIC")]:
        test_valid_protocol(
            f"Test 1.9.{cmd_id}: Komenda {cmd_name}",
            cmd_id, [0xA0, 0xA1, 0xA2], 0x10 + cmd_id,
            category="Valid Protocol - All Commands"
        )
        time.sleep(INTER_TEST_DELAY)
    
    # Test 1.10: Różne długości payload
    for size in [1, 2, 3, 5, 7, 10, 12, 15, 16]:
        test_valid_protocol(
            f"Test 1.10.{size}: Payload {size} bajtów",
            CMD_HOMING, list(range(0xB0, 0xB0 + size)), 0x20 + size,
            category="Valid Protocol - Various Sizes"
        )
        time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 2: WALIDACJA HEADERA - BŁĘDNY START_BYTE
    # ========================================
    print("\n" + "═"*70)
    print("🔍 KATEGORIA 2: WALIDACJA HEADERA - START_BYTE")
    print("═"*70)
    
    test_invalid_header(
        "Test 2.1: Błędny START_BYTE (0xAA zamiast 0x55)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        start_byte=0xAA,
        expected_error=WRONG_START_BYTE,
        category="Header Validation - START_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 2.2: Błędny START_BYTE (0x00)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        start_byte=0x00,
        expected_error=WRONG_START_BYTE,
        category="Header Validation - START_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 2.3: Błędny START_BYTE (0xFF)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        start_byte=0xFF,
        expected_error=WRONG_START_BYTE,
        category="Header Validation - START_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 3: WALIDACJA HEADERA - BŁĘDNY SLAVE_ID
    # ========================================
    print("\n" + "═"*70)
    print("🔍 KATEGORIA 3: WALIDACJA HEADERA - SLAVE_ID")
    print("═"*70)
    
    test_invalid_header(
        "Test 3.1: Błędny SLAVE_ID (0x02 zamiast 0x01)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        slave_id=0x02,
        expected_error=WRONG_SLAVE_ID,
        category="Header Validation - SLAVE_ID"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 3.2: Błędny SLAVE_ID (0x00)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        slave_id=0x00,
        expected_error=WRONG_SLAVE_ID,
        category="Header Validation - SLAVE_ID"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 3.3: Błędny SLAVE_ID (0xFF)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        slave_id=0xFF,
        expected_error=WRONG_SLAVE_ID,
        category="Header Validation - SLAVE_ID"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 4: WALIDACJA HEADERA - BŁĘDNA KOMENDA
    # ========================================
    print("\n" + "═"*70)
    print("🔍 KATEGORIA 4: WALIDACJA HEADERA - COMMAND")
    print("═"*70)
    
    test_invalid_header(
        "Test 4.1: Komenda poniżej zakresu (99)",
        99, [0x10, 0x20, 0x30], 0x01,
        expected_error=WRONG_COMMAND,
        category="Header Validation - COMMAND"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 4.2: Komenda powyżej zakresu (103)",
        103, [0x10, 0x20, 0x30], 0x01,
        expected_error=WRONG_COMMAND,
        category="Header Validation - COMMAND"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 4.3: Komenda = 0",
        0, [0x10, 0x20, 0x30], 0x01,
        expected_error=WRONG_COMMAND,
        category="Header Validation - COMMAND"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 4.4: Komenda = 255",
        255, [0x10, 0x20, 0x30], 0x01,
        expected_error=WRONG_COMMAND,
        category="Header Validation - COMMAND"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 5: WALIDACJA HEADERA - BŁĘDNY DATA_SIZE
    # ========================================
    print("\n" + "═"*70)
    print("🔍 KATEGORIA 5: WALIDACJA HEADERA - DATA_SIZE")
    print("═"*70)
    
    # Uwaga: test_invalid_header buduje header z len(payload), 
    # ale możemy nadpisać data_size w headerze ręcznie
    # Zmodyfikujmy funkcję testową
    
    def test_invalid_data_size(test_name, declared_size, actual_payload):
        """Test z niezgodnym data_size w headerze."""
        try:
            print(f"\n{'─'*70}")
            print(f"🧪 {test_name}")
            print(f"   Kategoria: Header Validation - DATA_SIZE")
            print(f"   Zadeklarowano: {declared_size}B, Rzeczywisty payload: {len(actual_payload)}B")
            print(f"{'─'*70}")
            
            # Handshake
            send_receive_cs([SYN_BYTE])
            time.sleep(DELAY_BETWEEN_STEPS)
            send_receive_cs([0xFF])
            time.sleep(DELAY_BETWEEN_STEPS)
            send_receive_cs([ACK_BYTE])
            time.sleep(DELAY_BETWEEN_STEPS)
            
            # BŁĘDNY HEADER (data_size > MAX lub niezgodny)
            length = HEADER_SIZE + declared_size
            header = [START_BYTE, length, SLAVE_ID, 0x01, CMD_HOMING, declared_size, END_BYTE]
            print(f"→ HEADER (data_size={declared_size}): {[f'0x{b:02X}' for b in header]}")
            send_receive_cs(header)
            time.sleep(DELAY_BETWEEN_STEPS)
            
            # VALIDATION
            print("← VALIDATION_CODE")
            resp = send_receive_cs([0xFF])
            validation = resp[0]
            val_name = get_validation_name(validation)
            
            print(f"  Otrzymano: {val_name} (0x{validation:02X})")
            
            if validation == WRONG_DATA_SIZE:
                print("  ✓ STM32 odrzucił błędny data_size")
                print("✅ Test PASSED")
                stats.add_result("Header Validation - DATA_SIZE", True)
                return True
            else:
                print("  ❌ STM32 nie wykrył błędnego data_size")
                print("❌ Test FAILED")
                stats.add_result("Header Validation - DATA_SIZE", False)
                return False
            
        except Exception as e:
            print(f"❌ Wyjątek: {e}")
            stats.add_result("Header Validation - DATA_SIZE", False)
            return False
    
    test_invalid_data_size(
        "Test 5.1: data_size = 17 (powyżej MAX_DATA_SIZE=16)",
        17, [0x10, 0x20, 0x30]
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_data_size(
        "Test 5.2: data_size = 255",
        255, [0x10, 0x20, 0x30]
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_data_size(
        "Test 5.3: data_size = 100",
        100, [0x10, 0x20, 0x30]
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 6: WALIDACJA HEADERA - BŁĘDNY END_BYTE
    # ========================================
    print("\n" + "═"*70)
    print("🔍 KATEGORIA 6: WALIDACJA HEADERA - END_BYTE")
    print("═"*70)
    
    test_invalid_header(
        "Test 6.1: Błędny END_BYTE (0x55 zamiast 0xAA)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        end_byte=0x55,
        expected_error=WRONG_END_BYTE,
        category="Header Validation - END_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 6.2: Błędny END_BYTE (0x00)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        end_byte=0x00,
        expected_error=WRONG_END_BYTE,
        category="Header Validation - END_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "Test 6.3: Błędny END_BYTE (0xFF)",
        CMD_HOMING, [0x10, 0x20, 0x30], 0x01,
        end_byte=0xFF,
        expected_error=WRONG_END_BYTE,
        category="Header Validation - END_BYTE"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 7: TESTY TIMEOUTU
    # ========================================
    print("\n" + "═"*70)
    print("⏱️  KATEGORIA 7: TESTY TIMEOUTU")
    print("═"*70)
    
    test_timeout(
        "Test 7.1: Timeout po SYN",
        'syn',
        category="Timeout Tests"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_timeout(
        "Test 7.2: Timeout po ACK",
        'ack',
        category="Timeout Tests"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_timeout(
        "Test 7.3: Timeout po HEADER",
        'header',
        category="Timeout Tests"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_timeout(
        "Test 7.4: Timeout po DATA",
        'data',
        category="Timeout Tests"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # KATEGORIA 8: STRESS TESTY
    # ========================================
    print("\n" + "═"*70)
    print("💪 KATEGORIA 8: STRESS TESTY")
    print("═"*70)
    
    # Test 8.1: Seria szybkich poprawnych ramek
    print("\n" + "─"*70)
    print("🧪 Test 8.1: Seria 10 szybkich poprawnych ramek")
    print("─"*70)
    success_count = 0
    for i in range(10):
        result = test_valid_protocol(
            f"  Ramka {i+1}/10",
            CMD_HOMING, [0xF0 + i, 0xF1 + i, 0xF2 + i], 0x80 + i,
            category="Stress Tests - Fast Frames"
        )
        if result:
            success_count += 1
        time.sleep(0.05)  # Krótka przerwa
    
    print(f"Wynik serii: {success_count}/10 ramek przeszło")
    
    # Test 8.2: Przemienne ramki poprawne/błędne
    print("\n" + "─"*70)
    print("🧪 Test 8.2: Przemienne ramki poprawne/błędne")
    print("─"*70)
    
    test_valid_protocol(
        "  Ramka poprawna #1",
        CMD_HOMING, [0x01, 0x02, 0x03], 0x90,
        category="Stress Tests - Alternating"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "  Ramka błędna #1 (START_BYTE)",
        CMD_HOMING, [0x01, 0x02, 0x03], 0x91,
        start_byte=0xFF,
        expected_error=WRONG_START_BYTE,
        category="Stress Tests - Alternating"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_valid_protocol(
        "  Ramka poprawna #2",
        CMD_MOVE_VIA_ANGLE, [0x04, 0x05, 0x06], 0x92,
        category="Stress Tests - Alternating"
    )
    time.sleep(INTER_TEST_DELAY)
    
    test_invalid_header(
        "  Ramka błędna #2 (COMMAND)",
        50, [0x04, 0x05, 0x06], 0x93,
        expected_error=WRONG_COMMAND,
        category="Stress Tests - Alternating"
    )
    time.sleep(INTER_TEST_DELAY)
    
    # ========================================
    # PODSUMOWANIE
    # ========================================
    stats.print_summary()

# ---------------------------
# URUCHOMIENIE TESTÓW
# ---------------------------
try:
    run_all_tests()

finally:
    spi.close()
    GPIO.cleanup()
    print("\n✅ Zasoby zwolnione.\n")