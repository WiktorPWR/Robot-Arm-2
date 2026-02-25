"""
robot_controller.py – Kontroler 6 osi ramienia robotycznego
=============================================================
Opakowuje bibliotekę spi_stm32 dla wszystkich 6 slave'ów STM32.
Każdy slave ma przypisany własny CS pin i READY pin.

Ścieżka biblioteki: Code/Rasberry-Pi-code/backend/robot/communication_library/

Użycie z app.py:
    from robot_controller import RobotController
    controller = RobotController()
    controller.home_all(on_axis_start=cb, on_axis_done=cb)
    controller.move_axis(axis=0, angle_deg=90.0)
    controller.estop()
    controller.close()
"""

import sys
import os
import threading
import logging

# ---------------------------------------------------------------------------
# Ścieżka do biblioteki komunikacyjnej
# ---------------------------------------------------------------------------
_LIB_PATH = os.path.join(os.path.dirname(__file__), "communication_library")
sys.path.insert(0, os.path.dirname(_LIB_PATH))  # dodaje backend/robot/ do path
if _LIB_PATH not in sys.path:
    sys.path.insert(0, _LIB_PATH)

from communication_library import SPIMaster
from communication_library.constants import (
    REG_HOMING, REG_MOVE_ANGLE, REG_EMERGENCY_STOP, REG_DIAG_STATUS,
)

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Konfiguracja pinów GPIO dla każdego slave'a
# ---------------------------------------------------------------------------
# Indeks = numer osi (0..5)
#   Oś 0 → STM32 #1 (Base,  NEMA 23 × 1)
#   Oś 1 → STM32 #2 (Axis2, NEMA 23 × 2)
#   Oś 2 → STM32 #3 (Axis3, NEMA 23 × 2)
#   Oś 3 → STM32 #4 (Axis4, NEMA 23 × 1)
#   Oś 4 → STM32 #5 (Axis5, NEMA 17 × 2)
#   Oś 5 → STM32 #6 (TCP,   NEMA 17 × 1)
CS_PINS    = [5,  6,  13, 19, 26, 21]
READY_PINS = [22, 23, 24, 25, 27, 17]


class RobotController:
    """
    Kontroler 6-osiowego ramienia robotycznego.

    Tworzy 6 instancji SPIMaster (po jednej na oś) i udostępnia
    operacje homing, move, estop, odczytu statusu i kątów.
    """

    NUM_AXES = 6

    def __init__(
        self,
        cs_pins:    list[int] = CS_PINS,
        ready_pins: list[int] = READY_PINS,
    ):
        if len(cs_pins) != self.NUM_AXES or len(ready_pins) != self.NUM_AXES:
            raise ValueError(f"Wymagane dokładnie {self.NUM_AXES} pinów CS i READY.")

        self._lock = threading.Lock()
        self._axes: list[SPIMaster] = []

        # Ostatnie wysłane kąty [°] – echo do UI zanim mamy enkodery
        self._last_angles: list[float] = [0.0] * self.NUM_AXES

        log.info("Inicjalizacja RobotController – tworzenie %d instancji SPIMaster...", self.NUM_AXES)
        for i in range(self.NUM_AXES):
            master = SPIMaster(cs_pin=cs_pins[i], ready_pin=ready_pins[i])
            self._axes.append(master)
            log.info("  Oś %d: CS=GPIO%d  READY=GPIO%d", i, cs_pins[i], ready_pins[i])

        log.info("RobotController gotowy.")

    # ------------------------------------------------------------------
    # HOMING
    # ------------------------------------------------------------------

    def home_axis(self, axis: int) -> bool:
        """
        Uruchamia homing na jednej osi i czeka na zakończenie (READY_PIN LOW).

        Po pomyślnym homingu zeruje zapamiętany kąt tej osi.
        """
        if not self._valid_axis(axis):
            return False

        log.info("[HOMING] Oś %d – start", axis)
        with self._lock:
            ok = self._axes[axis].write_homing(1)

        if not ok:
            log.error("[HOMING] Oś %d – błąd wysyłania komendy SPI", axis)
            return False

        ready = self._axes[axis].wait_ready(timeout=30.0)
        if not ready:
            log.warning("[HOMING] Oś %d – timeout READY_PIN", axis)
        else:
            self._last_angles[axis] = 0.0   # po homingu kąt = 0

        log.info("[HOMING] Oś %d – zakończony (ok=%s)", axis, ready)
        return ready

    def home_all(
        self,
        on_axis_start: callable = None,
        on_axis_done:  callable = None,
    ) -> list[bool]:
        """
        Homing wszystkich 6 osi sekwencyjnie (oś po osi, od 0 do 5).

        Parametry
        ----------
        on_axis_start : callable(axis: int)
            Wywoływany tuż przed homingiem każdej osi.
        on_axis_done  : callable(axis: int, success: bool)
            Wywoływany po zakończeniu homing każdej osi.
        """
        results = []
        for axis in range(self.NUM_AXES):
            if on_axis_start:
                on_axis_start(axis)
            ok = self.home_axis(axis)
            results.append(ok)
            if on_axis_done:
                on_axis_done(axis, ok)
        return results

    # ------------------------------------------------------------------
    # MOVE
    # ------------------------------------------------------------------

    def move_axis(self, axis: int, angle_deg: float) -> bool:
        """
        Wysyła kąt docelowy do jednej osi przez SPI.

        Konwersja: float [°] → uint32 [millistopnie], zachowuje 3 miejsca po przecinku.
        Zapamiętuje wysłany kąt jako ostatni znany kąt tej osi.
        """
        if not self._valid_axis(axis):
            return False

        angle_uint32 = int(round(angle_deg * 1000)) & 0xFFFFFFFF
        log.info("[MOVE] Oś %d → %.3f° (raw=%d)", axis, angle_deg, angle_uint32)

        with self._lock:
            ok = self._axes[axis].write_move_angle(angle_uint32)

        if ok:
            self._last_angles[axis] = angle_deg
        else:
            log.error("[MOVE] Oś %d – błąd SPI", axis)

        return ok

    def move_all(self, angles_deg: list[float]) -> list[bool]:
        """
        Wysyła kąty do wszystkich 6 osi sekwencyjnie.

        Parametry
        ----------
        angles_deg : list[float]
            6 kątów [°], indeks = numer osi.
        """
        if len(angles_deg) != self.NUM_AXES:
            raise ValueError(f"Wymagane {self.NUM_AXES} kątów, podano {len(angles_deg)}.")
        return [self.move_axis(i, a) for i, a in enumerate(angles_deg)]

    # ------------------------------------------------------------------
    # ODCZYT KĄTÓW
    # ------------------------------------------------------------------

    def read_angle(self, axis: int) -> tuple[bool, float]:
        """
        Odczytuje kąt osi z REG_MOVE_ANGLE przez SPI.

        Slave zwraca ostatni zapisany kąt (uint32 millistopnie → float °).
        Jeśli odczyt SPI nie powiedzie się, zwraca ostatni zapamiętany kąt.

        Zwraca
        ------
        (success: bool, angle_deg: float)
        """
        if not self._valid_axis(axis):
            return False, 0.0

        with self._lock:
            ok, angle_uint32 = self._axes[axis].read_move_angle()

        if not ok:
            return False, self._last_angles[axis]

        angle_deg = angle_uint32 / 1000.0
        self._last_angles[axis] = angle_deg
        return True, angle_deg

    def read_all_angles(self) -> list[tuple[bool, float]]:
        """
        Odczytuje kąty ze wszystkich 6 osi przez SPI.

        Zwraca
        ------
        list[(success, angle_deg)]  – po jednym wpisie na oś.
        """
        return [self.read_angle(i) for i in range(self.NUM_AXES)]

    def get_last_angles(self) -> list[float]:
        """Zwraca zapamiętane kąty bez SPI – szybki fallback dla UI."""
        return list(self._last_angles)

    # ------------------------------------------------------------------
    # EMERGENCY STOP
    # ------------------------------------------------------------------

    def estop(self) -> list[bool]:
        """Wysyła EMERGENCY STOP do wszystkich osi równolegle (wątki)."""
        log.warning("[ESTOP] Wysyłanie do wszystkich osi!")
        results = [False] * self.NUM_AXES
        threads = []

        def _stop(axis: int):
            try:
                ok = self._axes[axis].set_emergency_stop(True)
                results[axis] = ok
                log.info("[ESTOP] Oś %d – %s", axis, "OK" if ok else "FAIL")
            except Exception as e:
                log.error("[ESTOP] Oś %d – wyjątek: %s", axis, e)

        for i in range(self.NUM_AXES):
            t = threading.Thread(target=_stop, args=(i,), daemon=True)
            threads.append(t)
            t.start()
        for t in threads:
            t.join(timeout=3.0)

        return results

    def estop_clear(self) -> list[bool]:
        """Kasuje EMERGENCY STOP na wszystkich osiach."""
        log.info("[ESTOP] Kasowanie na wszystkich osiach")
        results = []
        for i in range(self.NUM_AXES):
            with self._lock:
                ok = self._axes[i].set_emergency_stop(False)
            results.append(ok)
        return results

    # ------------------------------------------------------------------
    # STATUS / DIAGNOSTYKA
    # ------------------------------------------------------------------

    def read_diag_status(self, axis: int) -> tuple[bool, int]:
        """Odczytuje REG_DIAG_STATUS z wybranej osi."""
        if not self._valid_axis(axis):
            return False, 0
        with self._lock:
            return self._axes[axis].read_diag_status()

    def read_all_diag_status(self) -> list[tuple[bool, int]]:
        """Odczytuje REG_DIAG_STATUS ze wszystkich osi."""
        return [self.read_diag_status(i) for i in range(self.NUM_AXES)]

    def read_emergency_stop(self, axis: int) -> tuple[bool, bool]:
        """Odczytuje stan REG_EMERGENCY_STOP z wybranej osi."""
        if not self._valid_axis(axis):
            return False, False
        with self._lock:
            return self._axes[axis].read_emergency_stop()

    def is_any_in_error(self) -> bool:
        """True jeśli którakolwiek oś ma aktywny estop."""
        for i in range(self.NUM_AXES):
            ok, active = self.read_emergency_stop(i)
            if ok and active:
                return True
        return False

    # ------------------------------------------------------------------
    # ZAMKNIĘCIE
    # ------------------------------------------------------------------

    def close(self) -> None:
        log.info("RobotController – zamykanie SPI...")
        for i, master in enumerate(self._axes):
            try:
                master.close()
            except Exception as e:
                log.warning("  Oś %d – błąd zamykania: %s", i, e)
        log.info("RobotController zamknięty.")

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------

    def _valid_axis(self, axis: int) -> bool:
        if not (0 <= axis < self.NUM_AXES):
            log.error("Nieprawidłowy numer osi: %d (zakres 0–%d)", axis, self.NUM_AXES - 1)
            return False
        return True