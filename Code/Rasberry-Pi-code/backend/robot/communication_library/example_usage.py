"""
example_usage.py – Przykłady użycia biblioteki spi_stm32
=========================================================
Uruchomienie:
    python3 example_usage.py

Wymagania:
    pip install spidev RPi.GPIO
    Skrypt musi być uruchomiony na Raspberry Pi z podłączonym STM32.
"""

import struct
import time

# Importuj bibliotekę zainstalowaną lokalnie lub z tego samego katalogu
from spi_stm32 import SPIMaster
from spi_stm32 import (
    REG_HOMING, REG_MOVE_ANGLE,
    REG_DIAG_CONTROL, REG_DIAG_STATUS, REG_EMERGENCY_STOP,
    OK_FRAME, WRONG_CRC_VALUE,
)
from spi_stm32.utils.log import CommunicationLog


# ==========================================================================
# Przykład 1 – Context manager (zalecany sposób użycia)
# ==========================================================================
def example_context_manager():
    print("\n=== Przykład 1: Context manager ===")

    with SPIMaster(verbose=True) as spi:
        # Start homing
        ok = spi.write_homing(1)
        print(f"Homing start: {'OK' if ok else 'FAIL'}")
        time.sleep(0.2)

        # Stop homing
        spi.write_homing(0)

        # Ustaw kąt 90°
        ok = spi.write_move_angle(90)
        print(f"Ustaw kąt 90°: {'OK' if ok else 'FAIL'}")

        # Odczytaj kąt
        ok, angle = spi.read_move_angle()
        if ok:
            print(f"Odczytany kąt: {angle}°")

        # Odczytaj status diagnostyczny
        ok, status = spi.read_diag_status()
        if ok:
            print(f"Status diagnostyczny: 0x{status:08X}")

    # Zasoby zwolnione automatycznie przez __exit__


# ==========================================================================
# Przykład 2 – Ręczne zarządzanie zasobami
# ==========================================================================
def example_manual():
    print("\n=== Przykład 2: Manualne zarządzanie zasobami ===")

    spi = SPIMaster()
    try:
        # Emergency stop
        spi.set_emergency_stop(True)
        print("Emergency stop AKTYWNY")
        time.sleep(0.5)

        ok, active = spi.read_emergency_stop()
        print(f"Stan emergency stop: {active}")

        spi.set_emergency_stop(False)
        print("Emergency stop NIEAKTYWNY")

    finally:
        spi.close()   # Zawsze zwalniaj zasoby


# ==========================================================================
# Przykład 3 – Round-trip WRITE + READ z weryfikacją
# ==========================================================================
def example_round_trip():
    print("\n=== Przykład 3: Round-trip WRITE + READ ===")

    test_angles = [0, 45, 90, 180, 270, 360, 0xFFFFFFFF]

    with SPIMaster() as spi:
        for angle in test_angles:
            # Zapis
            ok_w = spi.write_move_angle(angle)

            # Odczyt
            ok_r, readback = spi.read_move_angle()

            match = "✓" if (ok_w and ok_r and readback == angle) else "✗"
            print(f"  [{match}] Zapis={angle:>12}  Odczyt={readback:>12}")


# ==========================================================================
# Przykład 4 – Generyczne write/read z niestandardowym log
# ==========================================================================
def example_with_logging():
    print("\n=== Przykład 4: Zapis z logowaniem MOSI/MISO ===")

    log = CommunicationLog()

    with SPIMaster() as spi:
        ok, err_code = spi.write(
            REG_HOMING,
            bytes([0x01]),
            log=log,
        )
        print(f"Wynik: {'OK' if ok else 'FAIL'} (slave_err=0x{err_code:02X})")

    # Wyświetl dump komunikacji
    log.print_flow()


# ==========================================================================
# Przykład 5 – Obsługa błędów i odzysk slave'a
# ==========================================================================
def example_error_handling():
    print("\n=== Przykład 5: Obsługa błędów ===")

    with SPIMaster() as spi:
        # Spróbuj odczytać status przed sprawdzeniem gotowości
        if not spi.is_slave_ready():
            print("Slave nie jest gotowy, czekam...")
            if not spi.wait_ready(timeout=3.0):
                print("Timeout – próba odzyskania slave'a")
                if not spi.recover():
                    print("Nie udało się odblokować slave'a!")
                    return

        # Normalna operacja po odzysku
        ok, angle = spi.read_move_angle()
        if ok:
            print(f"Kąt po odzysku: {angle}°")
        else:
            print("Odczyt zakończony błędem")


# ==========================================================================
# Przykład 6 – Testy protokołu (używanie parametrów testowych)
# ==========================================================================
def example_protocol_tests():
    print("\n=== Przykład 6: Parametry testowe protokołu ===")

    from spi_stm32 import WRONG_START_BYTE, WRONG_CRC_VALUE

    with SPIMaster() as spi:
        # Test 1: Celowo błędny start_byte – slave powinien odrzucić
        ok, err = spi.write(
            REG_HOMING,
            bytes([0x01]),
            start_byte=0xAA,                    # Błędny start
            expected_validation=WRONG_START_BYTE,   # Oczekujemy odrzucenia
        )
        print(f"Test błędny start_byte: {'PASS' if ok else 'FAIL'}")

        time.sleep(0.2)

        # Test 2: Celowo błędne CRC headera
        ok, err = spi.write(
            REG_HOMING,
            bytes([0x01]),
            corrupt_header_crc=True,
            expected_validation=WRONG_CRC_VALUE,
        )
        print(f"Test błędne CRC headera: {'PASS' if ok else 'FAIL'}")

        time.sleep(0.2)

        # Test 3: Celowo błędne CRC danych
        ok, err = spi.write(
            REG_HOMING,
            bytes([0x01]),
            corrupt_data_crc=True,
        )
        # Przy corrupt_data_crc=True sukces oznacza, że slave wykrył błąd CRC
        print(f"Test błędne CRC danych: {'PASS' if ok else 'FAIL'}")


# ==========================================================================
# Uruchomienie wszystkich przykładów
# ==========================================================================
if __name__ == "__main__":
    # Odkomentuj przykłady które chcesz uruchomić:
    example_context_manager()
    # example_manual()
    # example_round_trip()
    # example_with_logging()
    # example_error_handling()
    # example_protocol_tests()
