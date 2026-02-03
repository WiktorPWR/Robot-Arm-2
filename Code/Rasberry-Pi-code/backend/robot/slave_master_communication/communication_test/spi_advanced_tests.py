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
spi.max_speed_hz = 500000 # 500kHz jest bezpieczne dla 3B+
spi.mode = 0b00
spi.no_cs = True

# ---------------------------
# KONSTANTY I KONFIGURACJA PROTOKOŁU
# ---------------------------
START_BYTE = 0x55
END_BYTE   = 0xAA
DELAY_BETWEEN_STEPS = 0.005 # 5ms przerwy między krokami (dla stabilności)

def send_receive_cs(data):
    """Wysyła dane i zwraca odpowiedź MISO."""
    GPIO.output(CS_PIN, GPIO.LOW)
    # Krótki czas na ustabilizowanie linii i reakcję EXTI w STM32
    time.sleep(0.0001) 
    
    resp = spi.xfer2(data)
    
    GPIO.output(CS_PIN, GPIO.HIGH)
    return resp

# ---------------------------
# STATYSTYKI TESTÓW
# ---------------------------
test_counter = 0
test_passed = 0
test_failed = 0

def run_full_protocol(command_id, payload):
    """Przechodzi przez całą maszynę stanów bez interwencji użytkownika."""
    global test_counter, test_passed, test_failed
    test_counter += 1
    
    try:
        print(f"--- Start sesji: CMD 0x{command_id:02X} ---")

        # 1. SYN
        send_receive_cs([0x97])
        time.sleep(DELAY_BETWEEN_STEPS)

        # 2. SYN_ACK (Odbieramy potwierdzenie od STM32)
        resp_syn_ack = send_receive_cs([0xFF])
        print(f"Handshake: STM32 returned 0x{resp_syn_ack[0]:02X}")
        time.sleep(DELAY_BETWEEN_STEPS)

        # 3. ACK
        send_receive_cs([0x99])
        time.sleep(DELAY_BETWEEN_STEPS)

        # 4. HEADER [START, ID, ?, LEN, END]
        data_len = len(payload)
        header = [START_BYTE, 0x01, command_id, data_len, END_BYTE]
        send_receive_cs(header)
        time.sleep(DELAY_BETWEEN_STEPS)

        # 5. DATA
        send_receive_cs(payload)
        time.sleep(DELAY_BETWEEN_STEPS)

        # 6. ECHO (Odbieramy odpowiedź)
        # Chcemy odebrać tyle bajtów, ile wysłaliśmy w danych + np. 2 bajty ramki
        echo_resp = send_receive_cs([0xFF] * (data_len + 2))
        print(f"Echo Data: {[hex(b) for b in echo_resp]}")
        time.sleep(DELAY_BETWEEN_STEPS)

        # 7. COMMIT
        send_receive_cs([0x11])
        
        print("--- Sesja zakończona sukcesem ---\n")
        test_passed += 1
        return echo_resp

    except Exception as e:
        print(f"❌ Błąd w transmisji: {e}")
        print("--- Sesja NIEUDANA ---\n")
        test_failed += 1
        return None

# ---------------------------
# GŁÓWNA PĘTLA PROGRAMU
# ---------------------------
try:
    print("="*60)
    print("TESTY PROTOKOŁU SPI - RÓŻNE KONFIGURACJE RAMEK")
    print("="*60)
    print()
    
    # TEST 1: Minimalna ramka - 1 bajt
    print("TEST 1: Minimalna ramka (1 bajt)")
    run_full_protocol(command_id=0x01, payload=[0x10])
    time.sleep(0.1)
    
    # TEST 2: Mała ramka - 3 bajty
    print("TEST 2: Mała ramka (3 bajty)")
    run_full_protocol(command_id=0x02, payload=[0x10, 0x20, 0x30])
    time.sleep(0.1)
    
    # TEST 3: Średnia ramka - 8 bajtów
    print("TEST 3: Średnia ramka (8 bajtów)")
    run_full_protocol(command_id=0x03, payload=[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88])
    time.sleep(0.1)
    
    # TEST 4: Duża ramka - 16 bajtów (maksymalny DATA_SIZE)
    print("TEST 4: Duża ramka (16 bajtów - maksymalny DATA_SIZE)")
    run_full_protocol(command_id=0x04, payload=list(range(0x10, 0x20)))
    time.sleep(0.1)
    
    # TEST 5: Dane same zera
    print("TEST 5: Dane same zera")
    run_full_protocol(command_id=0x05, payload=[0x00, 0x00, 0x00, 0x00])
    time.sleep(0.1)
    
    # TEST 6: Dane same jedynki
    print("TEST 6: Dane same jedynki (0xFF)")
    run_full_protocol(command_id=0x06, payload=[0xFF, 0xFF, 0xFF, 0xFF])
    time.sleep(0.1)
    
    # TEST 7: Wzór przemienny 0xAA, 0x55
    print("TEST 7: Wzór przemienny (0xAA, 0x55)")
    run_full_protocol(command_id=0x07, payload=[0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55])
    time.sleep(0.1)
    
    # TEST 8: Różne długości payloadu
    print("TEST 8: Payload 2 bajty")
    run_full_protocol(command_id=0x08, payload=[0x01, 0x02])
    time.sleep(0.1)
    
    # TEST 9: Payload 5 bajtów
    print("TEST 9: Payload 5 bajtów")
    run_full_protocol(command_id=0x09, payload=[0xA1, 0xA2, 0xA3, 0xA4, 0xA5])
    time.sleep(0.1)
    
    # TEST 10: Payload 10 bajtów
    print("TEST 10: Payload 10 bajtów")
    run_full_protocol(command_id=0x0A, payload=[0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9])
    time.sleep(0.1)
    
    # TEST 11: Bajty specjalne protokołu w danych
    print("TEST 11: Bajty specjalne protokołu jako dane")
    run_full_protocol(command_id=0x0B, payload=[0x55, 0xAA, 0x97, 0x98, 0x99])
    time.sleep(0.1)
    
    # TEST 12: Sekwencja inkrementalna
    print("TEST 12: Sekwencja inkrementalna (0x00-0x0F)")
    run_full_protocol(command_id=0x0C, payload=list(range(16)))
    time.sleep(0.1)
    
    # TEST 13: Sekwencja dekrementalna
    print("TEST 13: Sekwencja dekrementalna (0xFF-0xF0)")
    run_full_protocol(command_id=0x0D, payload=list(range(0xFF, 0xEF, -1)))
    time.sleep(0.1)
    
    # TEST 14: Wzór szachownicy
    print("TEST 14: Wzór szachownicy")
    run_full_protocol(command_id=0x0E, payload=[0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF])
    time.sleep(0.1)
    
    # TEST 15: Payload z powtarzającym się bajtem
    print("TEST 15: Powtarzający się bajt (0x42)")
    run_full_protocol(command_id=0x0F, payload=[0x42] * 12)
    time.sleep(0.1)
    
    # TEST 16: Różne długości - seria
    print("TEST 16: Seria różnych długości (1, 3, 6, 10, 15)")
    test_series = [
        [0xC1],
        [0xC2, 0xC3, 0xC4],
        [0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA],
        [0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9],
        list(range(0xE0, 0xEF))
    ]
    for i, p in enumerate(test_series):
        print(f"  Podseria {i+1}: {len(p)} bajtów")
        run_full_protocol(command_id=0x10 + i, payload=p)
        time.sleep(0.1)
    
    # TEST 17: Maksymalne wartości command_id
    print("TEST 17: Maksymalne wartości command_id (0xFF)")
    run_full_protocol(command_id=0xFF, payload=[0x99, 0x88, 0x77])
    time.sleep(0.1)
    
    # TEST 18: Kombinacja różnych wzorów
    print("TEST 18: Kombinacja wzorów")
    run_full_protocol(command_id=0x18, payload=[0x00, 0xFF, 0xAA, 0x55, 0x12, 0x34, 0x56, 0x78])
    time.sleep(0.1)
    
    # TEST 19: Wartości binarne
    print("TEST 19: Wartości binarne (potęgi dwójki)")
    run_full_protocol(command_id=0x19, payload=[0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80])
    time.sleep(0.1)
    
    # TEST 20: Wartości ASCII (znaki)
    print("TEST 20: Wartości ASCII (ABC123)")
    run_full_protocol(command_id=0x20, payload=[0x41, 0x42, 0x43, 0x31, 0x32, 0x33])
    time.sleep(0.1)
    
    # TEST 21-25: Szybkie pakiety jeden po drugim (stress test)
    print("TEST 21-25: Seria szybkich pakietów (stress test)")
    for i in range(5):
        print(f"  Szybki pakiet {i+1}/5")
        run_full_protocol(command_id=0x21 + i, payload=[0xF0 + i, 0xF1 + i, 0xF2 + i])
        time.sleep(0.05)  # Krótka przerwa
    
    print()
    print("="*60)
    print("WSZYSTKIE TESTY ZAKOŃCZONE")
    print("="*60)
    print()
    print("📊 STATYSTYKI TESTÓW:")
    print("-" * 60)
    print(f"  Wykonanych testów:     {test_counter}")
    print(f"  ✅ Udanych:            {test_passed}")
    print(f"  ❌ Nieudanych:         {test_failed}")
    print("-" * 60)
    
    if test_failed == 0:
        print(f"  🎉 Sukces: 100% testów przeszło!")
    else:
        success_rate = (test_passed / test_counter * 100) if test_counter > 0 else 0
        print(f"  📈 Wskaźnik sukcesu:   {success_rate:.1f}%")
    
    print("="*60)

finally:
    spi.close()
    GPIO.cleanup()
    print("Zasoby zwolnione.")