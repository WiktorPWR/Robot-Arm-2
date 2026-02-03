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
END_BYTE   = 0xAA

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

    
    
    print(f"  MISO <- {[f'0x{b:02X}' for b in resp]}")

    return resp

def wait_input(txt):
    input(f"\n{txt} (Enter)")

# ---------------------------
# PROTOKÓŁ
# ---------------------------
print("=== SPI Debugger STM32 ===")

# SYN
wait_input("Krok A: SYN")
syn = [0x97]
send_receive_cs(syn, "HANDSHAKE_SYN")

# SYN_ACK
wait_input("Krok B: SYN_ACK")
#time.sleep(0.001)
send_receive_cs([0xFF], "HANDSHAKE_SYN_ACK")

# ACK
wait_input("Krok C: ACK")
ack = [0x99]
send_receive_cs(ack,"HANDSHAKE_ACK")

# HEADER
wait_input("Krok 1: HEADER")
header = [START_BYTE, 0x01, 0x02, 0x03, END_BYTE]
send_receive_cs(header, "HEADER (STM32 tylko odbiera)")

# DATA
wait_input("Krok 2: DATA")
data = [0x10, 0x20, 0x30]
send_receive_cs(data, "DATA (STM32 tylko odbiera)")

# ECHO
wait_input("Krok 3: ECHO (STM32 odpowiada)")
#time.sleep(0.001)
send_receive_cs([0xFF] * (len(data) + 2), "ECHO (tu MA COŚ BYĆ)")

# COMMIT
wait_input("Krok 4: COMMIT")
send_receive_cs([0x11], "COMMIT")

print("\n=== KONIEC ===")

spi.close()
GPIO.cleanup()
