"""
transport.py – Niskopoziomowy transport SPI + GPIO
====================================================
Odpowiada za:
  • inicjalizację i zamknięcie magistrali SPI (spidev)
  • konfigurację GPIO (CS_PIN, READY_PIN)
  • pojedynczy transfer xfer2 z obsługą CS
  • oczekiwanie na READY_SLAVE_FLAG (GPIO22)
  • reset magistrali SPI

Wszystkie opóźnienia i numery pinów pochodzą z constants.py,
dzięki czemu można je łatwo dostosować bez modyfikacji logiki.
"""

import time
import spidev
import RPi.GPIO as GPIO

from ..constants import (
    CS_PIN, READY_PIN,
    CS_SETUP_TIME, READY_TIMEOUT_S, SPI_RESET_DELAY,
    SPI_BUS, SPI_DEVICE, SPI_SPEED_HZ, SPI_MODE,
)


class SPITransport:
    """
    Warstwa transportowa SPI z ręcznym sterowaniem CS i monitoringiem READY_PIN.

    Parametry
    ----------
    cs_pin : int
        Numer pinu BCM dla Chip Select (domyślnie CS_PIN z constants.py).
    ready_pin : int
        Numer pinu BCM dla READY_SLAVE_FLAG (domyślnie READY_PIN z constants.py).
    speed_hz : int
        Prędkość magistrali SPI w Hz.
    mode : int
        Tryb SPI (0–3).

    Przykład
    --------
    transport = SPITransport()
    resp = transport.xfer([0x97])   # wyślij SYN, odbierz 1 bajt
    transport.close()
    """

    def __init__(
        self,
        cs_pin:    int = CS_PIN,
        ready_pin: int = READY_PIN,
        speed_hz:  int = SPI_SPEED_HZ,
        mode:      int = SPI_MODE,
    ):
        self.cs_pin    = cs_pin
        self.ready_pin = ready_pin
        self._setup_gpio()
        self._spi = self._open_spi(speed_hz, mode)

    # ------------------------------------------------------------------
    # Inicjalizacja / zamknięcie
    # ------------------------------------------------------------------

    def _setup_gpio(self) -> None:
        """Konfiguruje piny GPIO (BCM, bez ostrzeżeń)."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cs_pin,    GPIO.OUT)
        GPIO.output(self.cs_pin,   GPIO.HIGH)   # CS nieaktywny (HIGH)
        GPIO.setup(self.ready_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _open_spi(self, speed_hz: int, mode: int) -> spidev.SpiDev:
        """Otwiera i konfiguruje urządzenie SPI."""
        dev = spidev.SpiDev()
        dev.open(SPI_BUS, SPI_DEVICE)
        dev.max_speed_hz = speed_hz
        dev.mode         = mode
        dev.no_cs        = True   # Ręczne sterowanie CS przez GPIO
        return dev

    def close(self) -> None:
        """Zamyka SPI i zwalnia GPIO."""
        try:
            self._spi.close()
        except Exception:
            pass
        GPIO.cleanup()

    # ------------------------------------------------------------------
    # Transfer danych
    # ------------------------------------------------------------------

    def xfer(self, data: list[int], log=None, step: str = "") -> list[int]:
        """
        Wykonuje pojedynczy transfer SPI z obsługą CS.

        Sekwencja:
          CS LOW → opóźnienie setup → xfer2(data) → CS HIGH

        Parametry
        ----------
        data : list[int]
            Bajty do wysłania (MOSI). Długość bufora = liczba odebranych bajtów.
        log : CommunicationLog | None
            Jeśli podany, transakcja zostanie zalogowana.
        step : str
            Opis kroku widoczny w logu.

        Zwraca
        ------
        list[int]
            Bajty odebrane od slave'a (MISO).
        """
        GPIO.output(self.cs_pin, GPIO.LOW)
        time.sleep(CS_SETUP_TIME)
        resp = self._spi.xfer2(list(data))
        GPIO.output(self.cs_pin, GPIO.HIGH)
        if log is not None:
            log.add(step, data, resp)
        return resp

    # ------------------------------------------------------------------
    # Sygnał gotowości slave'a
    # ------------------------------------------------------------------

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> bool:
        """
        Czeka, aż READY_PIN (GPIO22) opadnie do LOW.

        STM32 opuszcza pin po wykonaniu callbacku (HAL_SPI_RxCpltCallback).
        Metoda odpytuje pin co 1 ms.

        Parametry
        ----------
        timeout : float
            Maksymalny czas oczekiwania [s].

        Zwraca
        ------
        bool
            True  – pin opadł w czasie timeout.
            False – upłynął timeout bez reakcji slave'a.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if GPIO.input(self.ready_pin) == GPIO.LOW:
                return True
            time.sleep(0.001)
        return False

    def is_ready(self) -> bool:
        """Odczyt chwilowy stanu READY_PIN (True = LOW = slave gotowy)."""
        return GPIO.input(self.ready_pin) == GPIO.LOW

    # ------------------------------------------------------------------
    # Reset magistrali SPI
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """
        Resetuje magistralę SPI.

        Przydatne gdy slave jest zablokowany lub po nieoczekiwanym błędzie.
        Zamyka i ponownie otwiera spidev, a następnie wysyła 3×16 bajtów
        0x00 aby wyczyścić ewentualne buforowane dane.
        """
        try:
            self._spi.close()
        except Exception:
            pass
        time.sleep(SPI_RESET_DELAY)
        self._spi = self._open_spi(SPI_SPEED_HZ, SPI_MODE)
        # Flush – wyczyść potencjalne śmieci z linii MISO
        for _ in range(3):
            try:
                self._spi.xfer2([0x00] * 16)
            except Exception:
                pass
