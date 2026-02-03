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

def run_full_protocol(command_id, payload):
    """Przechodzi przez całą maszynę stanów bez interwencji użytkownika."""
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
        return echo_resp

    except Exception as e:
        print(f"Błąd w transmisji: {e}")
        return None

# ---------------------------
# GŁÓWNA PĘTLA PROGRAMU
# ---------------------------
try:
    # Przykład: Wyślij 3 różne pakiety jeden po drugim
    test_payloads = [
        [0x10, 0x20, 0x30],
        [0xAA, 0xBB, 0xCC, 0xDD],
        [0x01, 0x02]
    ]

    for i, p in enumerate(test_payloads):
        print(f"Próba {i+1}:")
        run_full_protocol(command_id=i, payload=p)
        time.sleep(0.1) # Oddech między całymi sesjami

finally:
    spi.close()
    GPIO.cleanup()
    print("Zasoby zwolnione.")