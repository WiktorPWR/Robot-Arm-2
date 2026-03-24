"""
ik_solver.py – Kinematyka odwrotna (IK) dla 6-osiowego ramienia robotycznego
==============================================================================
Implementacja oparta na parametrach Denavita-Hartenberga (DH).

Konwencja DH (klasyczna):
    Każde ogniwo i opisane jest przez 4 parametry:
        a_i     – długość ogniwa (mm)  – odległość między osiami Z po X
        d_i     – offset (mm)          – odległość między osiami X po Z
        alpha_i – skręt (rad)          – kąt między osiami Z po X
        theta_i – kąt stawu (rad)      – kąt między osiami X po Z (zmienna)

WAŻNE – uzupełnij przed użyciem:
    Wartości DH_PARAMS poniżej są PLACEHOLDERAMI.
    Zmierz rzeczywiste wymiary swojego ramienia i wpisz je tutaj.
    Opis jak to zrobić znajdziesz w komentarzach przy DH_PARAMS.

Zależności:
    pip install numpy
"""

import math
import logging
import numpy as np

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Parametry DH – UZUPEŁNIJ SWOIMI WARTOŚCIAMI
# ---------------------------------------------------------------------------
# Format każdego wiersza: [a (mm), d (mm), alpha (rad), theta_offset (rad)]
#
# theta_offset – stały offset kąta stawu (np. jeśli oś 0° mechanicznie
#                nie odpowiada 0° w DH). Dodawany do kąta podanego przez IK.
#
# Jak zmierzyć:
#   a_i     – odległość między osią Z(i-1) a Z(i), wzdłuż osi X(i-1)
#   d_i     – odległość między osią X(i-1) a X(i), wzdłuż osi Z(i)
#   alpha_i – kąt obrotu od Z(i-1) do Z(i) wokół X(i-1)
#
# Ogniwo | a [mm]  | d [mm]  | alpha [rad]     | theta_offset [rad]
DH_PARAMS = [
    # Oś 1 – Base (obrót w pionie, NEMA 23 × 1)
    [0.0,    100.0,   math.pi / 2,   0.0],   # TODO: wpisz rzeczywiste a, d
    # Oś 2 – Ramię (NEMA 23 × 2)
    [250.0,  0.0,     0.0,           0.0],   # TODO: wpisz rzeczywiste a
    # Oś 3 – Przedramię (NEMA 23 × 2)
    [250.0,  0.0,     0.0,           0.0],   # TODO: wpisz rzeczywiste a
    # Oś 4 – Nadgarstek rotacja (NEMA 23 × 1)
    [0.0,    0.0,     math.pi / 2,   0.0],
    # Oś 5 – Nadgarstek zgięcie (NEMA 17 × 2)
    [0.0,    0.0,    -math.pi / 2,   0.0],
    # Oś 6 – TCP (NEMA 17 × 1)
    [0.0,    80.0,    0.0,           0.0],   # TODO: wpisz d = długość końcówki
]

# Limity kątów stawów [stopnie] – [min, max]
# Uzupełnij zgodnie z mechanicznymi ograniczeniami osi.
JOINT_LIMITS_DEG = [
    [-170.0,  170.0],   # Oś 1
    [ -90.0,   90.0],   # Oś 2
    [-135.0,  135.0],   # Oś 3
    [-170.0,  170.0],   # Oś 4
    [-120.0,  120.0],   # Oś 5
    [-360.0,  360.0],   # Oś 6
]

# Pozycja domowa [stopnie] – używana gdy IK ma wiele rozwiązań
HOME_ANGLES_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Macierz transformacji DH dla jednego ogniwa
# ---------------------------------------------------------------------------

def _dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """
    Zwraca macierz transformacji 4×4 dla jednego ogniwa wg konwencji DH.

    Parametry
    ----------
    a, d, alpha – parametry DH ogniwa
    theta       – aktualny kąt stawu [rad]
    """
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct,  -st * ca,   st * sa,   a * ct],
        [st,   ct * ca,  -ct * sa,   a * st],
        [0.0,  sa,         ca,        d     ],
        [0.0,  0.0,        0.0,       1.0   ],
    ])


# ---------------------------------------------------------------------------
# Kinematyka prosta (FK) – dla weryfikacji rozwiązania IK
# ---------------------------------------------------------------------------

def forward_kinematics(angles_deg: list[float]) -> np.ndarray:
    """
    Oblicza macierz transformacji TCP względem bazy (kinematyka prosta).

    Parametry
    ----------
    angles_deg : list[float]
        Lista 6 kątów stawów [°].

    Zwraca
    ------
    np.ndarray (4×4)
        Macierz transformacji T_base_TCP.
    """
    T = np.eye(4)
    for i, (a, d, alpha, theta_off) in enumerate(DH_PARAMS):
        theta = math.radians(angles_deg[i]) + theta_off
        T = T @ _dh_matrix(a, d, alpha, theta)
    return T


# ---------------------------------------------------------------------------
# Kinematyka odwrotna – metoda numeryczna (Jacobian pseudo-inverse)
# ---------------------------------------------------------------------------

class IKSolver:
    """
    Solver kinematyki odwrotnej dla 6-DOF ramienia.

    Metoda: iteracyjna, Jacobian pseudo-inverse (Damped Least Squares).
    Działa dla dowolnej geometrii DH, nie wymaga rozwiązania analitycznego.

    Dla dobrze skonfigurowanych DH_PARAMS osiąga zbieżność w ~10–50 iteracjach.

    Parametry
    ----------
    max_iter : int
        Maks. liczba iteracji (domyślnie 200).
    tolerance_pos : float
        Tolerancja pozycji [mm] – warunek zatrzymania.
    tolerance_rot : float
        Tolerancja orientacji [rad] – warunek zatrzymania.
    damping : float
        Współczynnik tłumienia DLS (zapobiega osobliwościom Jacobianu).
    """

    def __init__(
        self,
        max_iter:      int   = 200,
        tolerance_pos: float = 0.1,    # mm
        tolerance_rot: float = 0.001,  # rad
        damping:       float = 0.05,
    ):
        self.max_iter      = max_iter
        self.tolerance_pos = tolerance_pos
        self.tolerance_rot = tolerance_rot
        self.damping       = damping
        self._dh           = DH_PARAMS
        self._limits       = [
            (math.radians(lo), math.radians(hi))
            for lo, hi in JOINT_LIMITS_DEG
        ]

    # ------------------------------------------------------------------
    # Publiczne API
    # ------------------------------------------------------------------

    def solve(
        self,
        x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        initial_angles_deg: list[float] | None = None,
    ) -> tuple[bool, list[float]]:
        """
        Oblicza kąty stawów dla zadanej pozycji i orientacji TCP.

        Parametry
        ----------
        x, y, z : float
            Pozycja TCP [mm] w układzie bazy.
        rx, ry, rz : float
            Orientacja TCP [°] – kąty Eulera ZYX (Roll-Pitch-Yaw).
        initial_angles_deg : list[float] | None
            Punkt startowy iteracji [°]. None = pozycja domowa.

        Zwraca
        ------
        (success: bool, angles_deg: list[float])
            success     – True jeśli znaleziono rozwiązanie w tolerancji.
            angles_deg  – lista 6 kątów [°]. Jeśli success=False,
                          zwraca najlepsze przybliżenie.
        """
        # Macierz docelowa
        T_target = self._pose_to_matrix(x, y, z, rx, ry, rz)

        # Punkt startowy
        if initial_angles_deg is None:
            q = [math.radians(a) for a in HOME_ANGLES_DEG]
        else:
            q = [math.radians(a) for a in initial_angles_deg]

        log.debug("[IK] Start: target=(%.1f, %.1f, %.1f) rot=(%.1f, %.1f, %.1f)",
                  x, y, z, rx, ry, rz)

        for iteration in range(self.max_iter):
            T_current = self._fk_rad(q)

            # Błąd pozycji i orientacji
            err_pos, err_rot = self._compute_error(T_current, T_target)
            err_vec = np.concatenate([err_pos, err_rot])

            err_norm_pos = np.linalg.norm(err_pos)
            err_norm_rot = np.linalg.norm(err_rot)

            # Warunek zbieżności
            if err_norm_pos < self.tolerance_pos and err_norm_rot < self.tolerance_rot:
                angles_deg = [math.degrees(qi) for qi in q]
                log.info("[IK] Zbieżność po %d iteracjach. Błąd: pos=%.4fmm rot=%.4frad",
                         iteration, err_norm_pos, err_norm_rot)
                return True, angles_deg

            # Jacobian numeryczny
            J = self._jacobian(q)

            # Damped Least Squares (DLS): dq = J^T (J J^T + λ²I)^{-1} err
            lam2 = self.damping ** 2
            A    = J @ J.T + lam2 * np.eye(6)
            dq   = J.T @ np.linalg.solve(A, err_vec)

            # Krok adaptacyjny
            step = min(1.0, 0.5 / (np.linalg.norm(dq) + 1e-9))
            q    = [qi + step * dqi for qi, dqi in zip(q, dq)]

            # Clamp do limitów
            q = [
                max(lo, min(hi, qi))
                for qi, (lo, hi) in zip(q, self._limits)
            ]

        # Nie osiągnięto zbieżności – zwróć najlepsze przybliżenie
        angles_deg = [math.degrees(qi) for qi in q]
        T_final = self._fk_rad(q)
        ep, er  = self._compute_error(T_final, T_target)
        log.warning("[IK] Brak zbieżności po %d iteracjach. "
                    "Błąd końcowy: pos=%.2fmm rot=%.4frad",
                    self.max_iter, np.linalg.norm(ep), np.linalg.norm(er))
        return False, angles_deg

    def verify(self, angles_deg: list[float], x: float, y: float, z: float) -> dict:
        """
        Sprawdza rozwiązanie IK przez FK i zwraca błąd pozycji.

        Zwraca słownik z polami: fk_pos, target_pos, error_mm.
        """
        T = forward_kinematics(angles_deg)
        fk_pos = T[:3, 3]
        target = np.array([x, y, z])
        err    = np.linalg.norm(fk_pos - target)
        return {
            "fk_pos":    fk_pos.tolist(),
            "target_pos": target.tolist(),
            "error_mm":  float(err),
        }

    # ------------------------------------------------------------------
    # Prywatne helpery
    # ------------------------------------------------------------------

    def _fk_rad(self, q_rad: list[float]) -> np.ndarray:
        """FK z kątami w radianach."""
        T = np.eye(4)
        for i, (a, d, alpha, theta_off) in enumerate(self._dh):
            T = T @ _dh_matrix(a, d, alpha, q_rad[i] + theta_off)
        return T

    def _pose_to_matrix(
        self, x, y, z, rx_deg, ry_deg, rz_deg
    ) -> np.ndarray:
        """
        Buduje macierz 4×4 z pozycji XYZ i kątów Eulera ZYX (RPY).
        rx=Roll (obrót wokół X), ry=Pitch (Y), rz=Yaw (Z).
        """
        rx = math.radians(rx_deg)
        ry = math.radians(ry_deg)
        rz = math.radians(rz_deg)

        # Macierz rotacji ZYX: R = Rz * Ry * Rx
        Rx = np.array([
            [1, 0,            0           ],
            [0, math.cos(rx), -math.sin(rx)],
            [0, math.sin(rx),  math.cos(rx)],
        ])
        Ry = np.array([
            [ math.cos(ry), 0, math.sin(ry)],
            [0,             1, 0            ],
            [-math.sin(ry), 0, math.cos(ry)],
        ])
        Rz = np.array([
            [math.cos(rz), -math.sin(rz), 0],
            [math.sin(rz),  math.cos(rz), 0],
            [0,             0,            1],
        ])
        R = Rz @ Ry @ Rx

        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = [x, y, z]
        return T

    def _compute_error(
        self, T_current: np.ndarray, T_target: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Oblicza błąd pozycji [mm] i orientacji [rad] między bieżącą a docelową pozą.
        Błąd orientacji wyrażony jako oś-kąt.
        """
        # Błąd pozycji
        err_pos = T_target[:3, 3] - T_current[:3, 3]

        # Błąd rotacji: R_err = R_current^T * R_target
        R_cur = T_current[:3, :3]
        R_tgt = T_target[:3, :3]
        R_err = R_cur.T @ R_tgt

        # Oś-kąt z macierzy rotacji
        trace = np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)
        angle = math.acos(trace)

        if abs(angle) < 1e-6:
            err_rot = np.zeros(3)
        else:
            axis = np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ]) / (2 * math.sin(angle))
            # Wyrażamy błąd jako wektor w układzie bazy (obrót przez R_cur)
            err_rot = R_cur @ (axis * angle)

        return err_pos, err_rot

    def _jacobian(self, q: list[float], delta: float = 1e-4) -> np.ndarray:
        """
        Numeryczny Jacobian 6×6 (różniczkowanie skończone).

        delta – krok różniczkowania [rad].
        """
        T0  = self._fk_rad(q)
        J   = np.zeros((6, len(q)))
        err0_pos, err0_rot = self._compute_error(T0, T0)   # = [0, 0, ...]

        for i in range(len(q)):
            q_p    = list(q)
            q_p[i] += delta
            T_p    = self._fk_rad(q_p)

            # Błąd pozycji
            dp = (T_p[:3, 3] - T0[:3, 3]) / delta

            # Błąd orientacji
            R0  = T0[:3, :3]
            Rp  = T_p[:3, :3]
            dR  = (Rp - R0) / delta
            # Prędkość kątowa ze skew-symmetric: w = [dR * R^T]_skew
            W   = dR @ R0.T
            dr  = np.array([W[2, 1], W[0, 2], W[1, 0]])

            J[:3, i] = dp
            J[3:, i] = dr

        return J


# ---------------------------------------------------------------------------
# Singleton – jedna instancja na cały proces
# ---------------------------------------------------------------------------
_solver: IKSolver | None = None


def get_solver() -> IKSolver:
    """Zwraca (lub tworzy) singleton IKSolver."""
    global _solver
    if _solver is None:
        _solver = IKSolver()
    return _solver


def solve_ik(
    x: float, y: float, z: float,
    rx: float, ry: float, rz: float,
    initial_angles_deg: list[float] | None = None,
) -> tuple[bool, list[float]]:
    """
    Skrócone API – rozwiązuje IK i zwraca (success, angles_deg).

    Parametry
    ----------
    x, y, z     – pozycja TCP [mm]
    rx, ry, rz  – orientacja TCP [°] (Roll-Pitch-Yaw / ZYX)
    initial_angles_deg – opcjonalny punkt startowy iteracji [°]

    Zwraca
    ------
    (success: bool, angles: list[float])
        success = False oznacza brak zbieżności – angles to najlepsze przybliżenie.
    """
    return get_solver().solve(x, y, z, rx, ry, rz, initial_angles_deg)