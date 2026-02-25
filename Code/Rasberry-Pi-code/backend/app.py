"""
app.py – Flask backend dla panelu sterowania ramieniem robotycznym
==================================================================
Struktura plików (relatywnie do tego pliku):
    app.py
    robot_controller.py
    ik_solver.py
    communication_library/
        spi_stm32/  ...
    templates/
        index.html
"""

import atexit
import threading
import logging

from flask import Flask, render_template, request, jsonify
from robot.robot_controller import RobotController
from robot.ik_calculation.ik_solver import solve_ik

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger(__name__)

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Inicjalizacja kontrolera
# ---------------------------------------------------------------------------
controller = RobotController()

# ---------------------------------------------------------------------------
# STAN ROBOTA (GLOBALNY)
# ---------------------------------------------------------------------------
robot_state = {
    "status":               "IDLE",         # IDLE | MOVING | HOMING | ERROR
    "axes_homed":           [False] * 6,    # czy oś przeszła homing
    "current_homing_axis":  None,           # aktualnie homowana oś (int | None)
    "angles":               [0.0] * 6,      # ostatnie kąty osi [°]
    "angles_ok":            [False] * 6,    # czy odczyt SPI kąta był OK
}

state_lock = threading.Lock()


# ===========================================================================
# ROUTES
# ===========================================================================

@app.route("/")
def index():
    return render_template("index.html")


# ---------------------------------------------------------------------------
# /status – polling z UI (co 200 ms)
# ---------------------------------------------------------------------------
@app.route("/status")
def status():
    # Odczyt kątów przez SPI przy każdym /status
    angles_raw = controller.read_all_angles()   # list[(ok, angle_deg)]

    with state_lock:
        for i, (ok, angle) in enumerate(angles_raw):
            robot_state["angles"][i]    = round(angle, 2)
            robot_state["angles_ok"][i] = ok

        return jsonify({
            "status":               robot_state["status"],
            "axes_homed":           robot_state["axes_homed"],
            "current_homing_axis":  robot_state["current_homing_axis"],
            "homed":                all(robot_state["axes_homed"]),
            "angles":               list(robot_state["angles"]),
            "angles_ok":            list(robot_state["angles_ok"]),
        })


# ---------------------------------------------------------------------------
# /move
# ---------------------------------------------------------------------------
@app.route("/move", methods=["POST"])
def move():
    data = request.get_json()

    try:
        x  = float(data["x"])
        y  = float(data["y"])
        z  = float(data["z"])
        rx = float(data["rx"])
        ry = float(data["ry"])
        rz = float(data["rz"])
    except (KeyError, ValueError) as e:
        return jsonify({"ok": False, "error": f"Nieprawidłowe dane: {e}"}), 400

    log.info("[/move] XYZ=(%.2f, %.2f, %.2f)  RPY=(%.2f, %.2f, %.2f)", x, y, z, rx, ry, rz)

    with state_lock:
        if robot_state["status"] not in ("IDLE",):
            return jsonify({"ok": False, "error": f"Robot zajęty: {robot_state['status']}"}), 409
        current_angles = list(robot_state["angles"])

    threading.Thread(
        target=_execute_move,
        args=(x, y, z, rx, ry, rz, current_angles),
        daemon=True,
    ).start()
    return jsonify({"ok": True})


def _execute_move(x, y, z, rx, ry, rz, current_angles):
    """Wątek wykonujący IK + wysłanie kątów przez SPI."""
    with state_lock:
        robot_state["status"] = "MOVING"

    try:
        # -------------------------------------------------------------------
        # Kinematyka odwrotna
        # Przekazujemy aktualne kąty jako punkt startowy iteracji IK
        # (szybsza zbieżność, bliższe rozwiązanie do bieżącej pozy).
        # -------------------------------------------------------------------
        ik_ok, angles = solve_ik(x, y, z, rx, ry, rz, initial_angles_deg=current_angles)

        if not ik_ok:
            log.warning("[MOVE] IK bez zbieżności – używam najlepszego przybliżenia")
            # Nie przerywamy ruchu – wysyłamy najlepsze przybliżenie.
            # Zmień na `return _set_error()` jeśli chcesz blokować ruch bez zbieżności.

        log.info("[MOVE] IK → kąty: %s", [f"{a:.2f}" for a in angles])

        # -------------------------------------------------------------------
        # Wyślij kąty przez SPI
        # -------------------------------------------------------------------
        results = controller.move_all(angles)

        if not all(results):
            failed = [i for i, ok in enumerate(results) if not ok]
            log.error("[MOVE] Błąd SPI na osiach: %s", failed)
            _set_error()
            return

    except Exception as e:
        log.exception("[MOVE] Wyjątek: %s", e)
        _set_error()
        return

    with state_lock:
        if robot_state["status"] != "ERROR":
            robot_state["status"] = "IDLE"


# ---------------------------------------------------------------------------
# /home
# ---------------------------------------------------------------------------
@app.route("/home", methods=["POST"])
def home():
    with state_lock:
        if robot_state["status"] not in ("IDLE", "ERROR"):
            return jsonify({"ok": False, "error": f"Robot zajęty: {robot_state['status']}"}), 409

    threading.Thread(target=_execute_homing, daemon=True).start()
    return jsonify({"ok": True})


def _execute_homing():
    """Wątek wykonujący homing sekwencyjnie oś po osi."""
    with state_lock:
        robot_state["status"]      = "HOMING"
        robot_state["axes_homed"]  = [False] * 6

    def on_axis_start(axis: int):
        with state_lock:
            if robot_state["status"] == "ERROR":
                return
            robot_state["current_homing_axis"] = axis
        log.info("[HOMING] Rozpoczynam oś %d", axis)

    def on_axis_done(axis: int, success: bool):
        with state_lock:
            robot_state["axes_homed"][axis] = success
            if success:
                robot_state["angles"][axis]    = 0.0
                robot_state["angles_ok"][axis] = True
        if not success:
            log.error("[HOMING] Oś %d – FAIL", axis)

    try:
        results = controller.home_all(
            on_axis_start=on_axis_start,
            on_axis_done=on_axis_done,
        )
    except Exception as e:
        log.exception("[HOMING] Wyjątek: %s", e)
        _set_error()
        return

    with state_lock:
        robot_state["current_homing_axis"] = None
        if robot_state["status"] == "ERROR":
            return
        if all(results):
            robot_state["status"] = "IDLE"
            log.info("[HOMING] Wszystkie osie skalibrowane.")
        else:
            robot_state["status"] = "ERROR"
            log.error("[HOMING] Nieudany na osiach: %s", [i for i, ok in enumerate(results) if not ok])


# ---------------------------------------------------------------------------
# /estop
# ---------------------------------------------------------------------------
@app.route("/estop", methods=["POST"])
def estop():
    log.warning("[/estop] EMERGENCY STOP!")

    with state_lock:
        robot_state["status"]              = "ERROR"
        robot_state["current_homing_axis"] = None

    try:
        results = controller.estop()
        failed  = [i for i, ok in enumerate(results) if not ok]
        if failed:
            log.error("[ESTOP] Nie zatrzymano osi: %s", failed)
        else:
            log.info("[ESTOP] Wszystkie osie zatrzymane.")
    except Exception as e:
        log.exception("[ESTOP] Wyjątek: %s", e)

    return jsonify({"ok": True})


@app.route("/move_axis", methods=["POST"])
def move_axis():
    """Wysyła kąt bezpośrednio do jednej osi (sterowanie ręczne z panelu osi)."""
    data = request.get_json()
    try:
        axis  = int(data["axis"])
        angle = float(data["angle"])
    except (KeyError, ValueError) as e:
        return jsonify({"ok": False, "error": f"Nieprawidłowe dane: {e}"}), 400

    with state_lock:
        if robot_state["status"] not in ("IDLE",):
            return jsonify({"ok": False, "error": f"Robot zajęty: {robot_state['status']}"}), 409

    log.info("[/move_axis] Oś %d → %.3f°", axis, angle)
    ok = controller.move_axis(axis, angle)
    return jsonify({"ok": ok})


@app.route("/estop_clear", methods=["POST"])
def estop_clear():
    """Kasuje EMERGENCY STOP i przywraca IDLE. Wywołaj po potwierdzeniu przez operatora."""
    log.info("[/estop_clear] Kasowanie emergency stop")
    try:
        controller.estop_clear()
    except Exception as e:
        log.exception("[ESTOP_CLEAR] Wyjątek: %s", e)
        return jsonify({"ok": False, "error": str(e)}), 500

    with state_lock:
        robot_state["status"] = "IDLE"

    return jsonify({"ok": True})


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def _set_error():
    with state_lock:
        robot_state["status"]              = "ERROR"
        robot_state["current_homing_axis"] = None


# ===========================================================================
# CLEANUP
# ===========================================================================
@atexit.register
def _shutdown():
    log.info("Zamykanie kontrolera SPI...")
    controller.close()


# ===========================================================================
# START
# ===========================================================================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)