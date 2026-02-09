import spidev
import RPi.GPIO as GPIO
import time

# ---------------------------
# KONFIGURACJA GPIO (ręczny CS)
# ---------------------------
CS_PIN = 5   # BCM 5 = fizyczny pin 29
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
# KONSTANTY
# ---------------------------
START_BYTE = 0x55
SLAVE_ID = 0x01
END_BYTE = 0xAA
SYN_BYTE = 0x97
SYN_ACK_BYTE = 0x98
ACK_BYTE = 0x99

# Kody błędów walidacji
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

# ---------------------------
# FUNKCJE
# ---------------------------
def send_receive_cs(data, label=""):
    print(f"\n[{label}]")
    print(f"  MOSI -> {[f'0x{b:02X}' for b in data]}")

    GPIO.output(CS_PIN, GPIO.LOW)
    time.sleep(0.001)
    
    resp = spi.xfer2(data)

    GPIO.output(CS_PIN, GPIO.HIGH)
    time.sleep(0.001)
    
    print(f"  MISO <- {[f'0x{b:02X}' for b in resp]}")

    return resp

def wait_input(txt):
    input(f"\n{txt} (Enter)")

def build_header(command, data_size, frame_id=0x01):
    """
    Buduje header zgodnie ze strukturą Header_t:
    - start_byte
    - length (całkowita długość ramki = HEADER_SIZE + data_size)
    - slave_id
    - frame_id
    - command
    - data_size
    - end_byte
    """
    HEADER_SIZE = 7  # sizeof(Header_t)
    length = HEADER_SIZE + data_size
    
    return [
        START_BYTE,    # start_byte
        length,        # length
        SLAVE_ID,      # slave_id
        frame_id,      # frame_id
        command,       # command
        data_size,     # data_size
        END_BYTE       # end_byte
    ]

# ---------------------------
# PROTOKÓŁ
# ---------------------------
print("=== SPI Debugger STM32 - Nowy Protokół ===")

# === HANDSHAKE ===
# SYN
wait_input("Krok A: SYN")
syn = [SYN_BYTE]
send_receive_cs(syn, "HANDSHAKE_SYN")

# SYN_ACK
wait_input("Krok B: SYN_ACK (odbierz)")
resp = send_receive_cs([0xFF], "HANDSHAKE_SYN_ACK")
if resp[0] == SYN_ACK_BYTE:
    print(f"  ✓ SYN_ACK poprawny (0x{SYN_ACK_BYTE:02X})")
else:
    print(f"  ✗ SYN_ACK niepoprawny! Oczekiwano 0x{SYN_ACK_BYTE:02X}, otrzymano 0x{resp[0]:02X}")

# ACK
wait_input("Krok C: ACK")
ack = [ACK_BYTE]
send_receive_cs(ack, "HANDSHAKE_ACK")

# === RAMKA DANYCH ===
# HEADER
wait_input("Krok 1: HEADER")
command = CMD_HOMING  # lub CMD_MOVE_VIA_ANGLE, CMD_DIAGNOSTIC
data_size = 3  # ilość bajtów danych
frame_id = 0x42

header = build_header(command, data_size, frame_id)
print(f"  Header: start=0x{header[0]:02X}, len={header[1]}, slave=0x{header[2]:02X}, "
      f"frame=0x{header[3]:02X}, cmd={header[4]}, data_size={header[5]}, end=0x{header[6]:02X}")
send_receive_cs(header, "HEADER")

# VALIDATION CODE
wait_input("Krok 2: VALIDATION CODE (odbierz)")
resp = send_receive_cs([0xFF], "VALIDATION_CODE")
validation_code = resp[0]

if validation_code == VALIDATION_OK:
    print(f"  ✓ Walidacja OK (0x{VALIDATION_OK:02X})")
    
    # DATA
    wait_input("Krok 3: DATA")
    data = [0x10, 0x20, 0x30]  # musi być data_size bajtów
    send_receive_cs(data, "DATA")
    
    # ECHO
    wait_input("Krok 4: ECHO (odbierz)")
    # Echo zawiera: start_byte + command + slave_id + frame_id + data + end_byte
    echo_size = 1 + 1 + 1 + 1 + data_size + 1  # = 8 dla data_size=3
    resp = send_receive_cs([0xFF] * echo_size, "ECHO")
    print(f"  Echo: start=0x{resp[0]:02X}, cmd={resp[1]}, slave=0x{resp[2]:02X}, "
          f"frame=0x{resp[3]:02X}, data={[f'0x{b:02X}' for b in resp[4:4+data_size]]}, "
          f"end=0x{resp[4+data_size]:02X}")
    
    # COMMIT
    wait_input("Krok 5: COMMIT")
    commit = [0x11]  # dowolny bajt jako potwierdzenie
    send_receive_cs(commit, "COMMIT")
    
else:
    error_names = {
        WRONG_START_BYTE: "WRONG_START_BYTE",
        WRONG_SLAVE_ID: "WRONG_SLAVE_ID",
        WRONG_COMMAND: "WRONG_COMMAND",
        WRONG_DATA_SIZE: "WRONG_DATA_SIZE",
        WRONG_END_BYTE: "WRONG_END_BYTE"
    }
    error_name = error_names.get(validation_code, f"UNKNOWN(0x{validation_code:02X})")
    print(f"  ✗ Walidacja FAILED: {error_name}")
    print("  Komunikacja przerwana, STM32 wrócił do WAIT_SYN")

print("\n=== KONIEC ===")

spi.close()
GPIO.cleanup()