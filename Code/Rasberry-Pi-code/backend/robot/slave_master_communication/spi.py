import RPi.GPIO as GPIO
import spidev
import time
from protocol import SlaveMasterProtocol

# ===============================
# SPI CONFIG
# ===============================
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 500_000
spi.mode = 0b00

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

SLAVE_PINS = [5, 6, 13, 19, 26, 21]

for pin in SLAVE_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

# ===============================
# FRAME CHECK
# ===============================
def check_received_frame(sent_cmd, received_frame) -> bool:
    """
    Sprawdza:
    - start / end
    - slave_id
    - command
    - CRC
    """
    if not SlaveMasterProtocol.validate_frame(received_frame):
        print("[SPI] CRC / FRAME ERROR")
        return False

    if received_frame[1] != sent_cmd[1]:
        print("[SPI] SLAVE ID mismatch")
        return False

    if received_frame[2] != sent_cmd[2]:
        print("[SPI] COMMAND mismatch")
        return False

    return True

# ===============================
# SEND COMMAND FLOW
# ===============================
def send_command(slave_id, command, data, timeout=0.1):
    """
    Flow komunikacji SPI z slave:
    1️⃣ Wyślij CMD
    2️⃣ Polling statusu od slave
    3️⃣ Wyślij GO / pozwolenie na wykonanie
    """

    if slave_id < 0 or slave_id >= len(SLAVE_PINS):
        raise ValueError("Nieprawidłowy slave_id")

    cs = SLAVE_PINS[slave_id]

    # --- FRAME 1: CMD ---
    cmd_frame = SlaveMasterProtocol.create_frame(
        slave_id=slave_id,
        command=command,
        data=data
    )

    GPIO.output(cs, GPIO.LOW)
    spi.xfer2(cmd_frame)  # wysyłamy CMD

    start = time.time()

    # --- FRAME 2: POLLING STATUS ---
    while time.time() - start < timeout:
        rx = spi.xfer2([0x00] * SlaveMasterProtocol.RESPONSE_LEN)

        if not SlaveMasterProtocol.validate_frame(rx):
            continue

        status = rx[2]  # STATUS W RAMCE

        if status == SlaveMasterProtocol.STATUS_BUSY:
            continue

        if status == SlaveMasterProtocol.STATUS_ERROR:
            GPIO.output(cs, GPIO.HIGH)
            print(f"[SPI] Slave {slave_id} ERROR")
            return False

        if status == SlaveMasterProtocol.STATUS_READY:
            # Slave gotowy do wykonania
            break
    else:
        GPIO.output(cs, GPIO.HIGH)
        print(f"[SPI] TIMEOUT waiting for READY from Slave {slave_id}")
        return False

    # --- FRAME 3: GO / EXECUTE ---
    exec_frame = SlaveMasterProtocol.create_flag_frame(
        slave_id=slave_id,
        flag=SlaveMasterProtocol.FLAG_GO
    )
    spi.xfer2(exec_frame)  # wysyłamy pozwolenie na wykonanie
    GPIO.output(cs, GPIO.HIGH)

    print(f"[SPI] Slave {slave_id} EXECUTE sent")
    return True

# ===============================
# GO / ABORT FLAG (osobna funkcja)
# ===============================
def send_go_flag(slave_id, allow: bool):
    cs = SLAVE_PINS[slave_id]

    flag = SlaveMasterProtocol.FLAG_GO if allow else SlaveMasterProtocol.FLAG_ABORT

    frame = SlaveMasterProtocol.create_flag_frame(
        slave_id=slave_id,
        flag=flag
    )

    GPIO.output(cs, GPIO.LOW)
    spi.xfer2(frame)
    GPIO.output(cs, GPIO.HIGH)
