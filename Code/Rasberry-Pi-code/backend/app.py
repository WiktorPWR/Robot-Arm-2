from flask import Flask, render_template, request, jsonify
import threading
import time

app = Flask(__name__)

# ===============================
# STAN ROBOTA (GLOBALNY)
# ===============================
robot_state = {
    "status": "IDLE",                 # IDLE | MOVING | HOMING | ERROR
    "axes_homed": [False]*6,           # status osi
    "current_homing_axis": None,       # aktualnie homowana oś
}

state_lock = threading.Lock()


# ===============================
# ROUTES
# ===============================
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/status")
def status():
    with state_lock:
        return jsonify({
            "status": robot_state["status"],
            "axes_homed": robot_state["axes_homed"],
            "current_homing_axis": robot_state["current_homing_axis"],
            "homed": all(robot_state["axes_homed"])
        })


@app.route("/move", methods=["POST"])
def move():
    data = request.get_json()

    x = data["x"]
    y = data["y"]
    z = data["z"]
    rx = data["rx"]
    ry = data["ry"]
    rz = data["rz"]

    print(f"[MOVE] XYZ=({x},{y},{z}) RPY=({rx},{ry},{rz})")

    # ===============================
    # TODO: Tutaj wywołujesz funkcję z controller.py do obliczenia kątów / IK
    # np: angles = controller.solve_ik(x, y, z, rx, ry, rz)
    # ===============================
    
    # ===============================
    # TODO: Tutaj wysyłasz obliczone kąty do SPI/UART
    # np: spi.send_move(angles)
    # ===============================

    threading.Thread(target=simulate_move, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/home", methods=["POST"])
def home():
    # ===============================
    # TODO: Tutaj wywołujesz funkcję homingu w controller.py
    # np: controller.start_homing()
    # ===============================

    # ===============================
    # TODO: Tutaj wyślij komendę homingu do SPI/UART
    # np: spi.send_homing()
    # ===============================

    threading.Thread(target=simulate_homing, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/estop", methods=["POST"])
def estop():
    with state_lock:
        robot_state["status"] = "ERROR"
        robot_state["current_homing_axis"] = None

    print("[EMERGENCY STOP]")

    # ===============================
    # TODO: Tutaj wyślij komendę estop do SPI/UART
    # np: spi.send_estop()
    # ===============================

    return jsonify({"ok": True})


# ===============================
# SYMULACJE (do UI)
# ===============================
def simulate_move():
    with state_lock:
        if robot_state["status"] == "ERROR":
            return
        robot_state["status"] = "MOVING"

    time.sleep(2.5)  # symulacja ruchu

    with state_lock:
        if robot_state["status"] != "ERROR":
            robot_state["status"] = "IDLE"


def simulate_homing():
    with state_lock:
        robot_state["status"] = "HOMING"
        robot_state["axes_homed"] = [False]*6

    for axis in range(6):
        with state_lock:
            if robot_state["status"] == "ERROR":
                return
            robot_state["current_homing_axis"] = axis

        time.sleep(1.0)  # symulacja homingu osi

        with state_lock:
            robot_state["axes_homed"][axis] = True

    with state_lock:
        robot_state["current_homing_axis"] = None
        robot_state["status"] = "IDLE"


# ===============================
# START FLASK
# ===============================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
