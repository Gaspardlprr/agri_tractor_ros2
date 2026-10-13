"""Mathématiques du guidon et du calibrage, sans dépendance ROS (testable seul).

Modèle : l'opérateur pousse le guidon latéralement avec une force F (axe Y du robot,
+ = vers la gauche) au point r_h = (hx, hy, hz) exprimé dans base_link, dont l'origine
est le centre de masse. Une force appliquée hors du CdM équivaut à :
    - la même force F appliquée au CdM
    - plus un couple  tau = r_h x F
Gazebo (ApplyLinkWrench) attend force ET couple dans le repère MONDE, d'où la
rotation par l'orientation courante du robot.

Avec hx < 0 (guidon à l'arrière) : pousser le guidon vers la gauche (F > 0) donne
tau_z = hx * F < 0, donc le tracteur tourne à DROITE. C'est le comportement réel
d'un motoculteur : on pousse les mancherons du côté opposé au virage.
"""
import math
from dataclasses import dataclass


@dataclass
class Wrench:
    force: tuple
    torque: tuple


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def quat_rotate(q, v):
    """Tourne le vecteur v par le quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    # v' = v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
    u = (x, y, z)
    uv = cross(u, v)
    uuv = cross(u, uv)
    return (v[0] + 2.0 * (w * uv[0] + uuv[0]),
            v[1] + 2.0 * (w * uv[1] + uuv[1]),
            v[2] + 2.0 * (w * uv[2] + uuv[2]))


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def handle_wrench_body(f_lat, handle_pos):
    """Force latérale au guidon -> torseur équivalent au CdM, repère robot."""
    force = (0.0, f_lat, 0.0)
    return Wrench(force=force, torque=cross(handle_pos, force))


def handle_wrench_world(f_lat, handle_pos, orientation):
    """Même torseur exprimé dans le repère monde (ce qu'attend ApplyLinkWrench)."""
    wb = handle_wrench_body(f_lat, handle_pos)
    return Wrench(force=quat_rotate(orientation, wb.force),
                  torque=quat_rotate(orientation, wb.torque))


def torque_to_handle_force(tau_z, handle_x):
    """Force latérale à appliquer au guidon pour obtenir un couple de lacet tau_z."""
    if abs(handle_x) < 1e-6:
        raise ValueError('handle_x ne peut pas être nul (guidon sur le CdM)')
    return tau_z / handle_x


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class YawRateController:
    """Couple de lacet = anticipation + PI sur la vitesse de lacet mesurée.

    tau = k_ff * w_cmd + kp * (w_cmd - w_mes) + ki * integrale(erreur)
    L'intégrale est gelée quand la force guidon sature (anti-windup).
    """

    def __init__(self, k_ff, kp, ki, i_limit, closed_loop=True):
        self.k_ff, self.kp, self.ki = k_ff, kp, ki
        self.i_limit = i_limit
        self.closed_loop = closed_loop
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, w_cmd, w_meas, dt, saturated_sign=0):
        tau = self.k_ff * w_cmd
        if not self.closed_loop:
            return tau
        err = w_cmd - w_meas
        # anti-windup : on n'intègre pas dans le sens qui aggrave la saturation
        if not (saturated_sign != 0 and math.copysign(1.0, err) == saturated_sign):
            self.integral = clamp(self.integral + err * dt, -self.i_limit, self.i_limit)
        return tau + self.kp * err + self.ki * self.integral


# ---------------------------------------------------------------- calibrage

def fit_circle(points):
    """Ajustement de cercle par moindres carrés (méthode de Kasa).

    points : liste de (x, y). Retourne (cx, cy, rayon). Résout
    x^2 + y^2 + D x + E y + F = 0 par équations normales 3x3.
    """
    n = len(points)
    if n < 3:
        raise ValueError('au moins 3 points')
    sxx = sxy = syy = sx = sy = 0.0
    sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sxx += x * x
        sxy += x * y
        syy += y * y
        sx += x
        sy += y
        sxz += x * z
        syz += y * z
        sz += z
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    b = [-sxz, -syz, -sz]
    d, e, f = _solve3(a, b)
    cx, cy = -d / 2.0, -e / 2.0
    r2 = cx * cx + cy * cy - f
    if r2 <= 0:
        raise ValueError('points alignés : pas de cercle')
    return cx, cy, math.sqrt(r2)


def _solve3(a, b):
    m = [row[:] + [bv] for row, bv in zip(a, b)]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(m[r][i]))
        if abs(m[p][i]) < 1e-12:
            raise ValueError('système singulier (points alignés ?)')
        m[i], m[p] = m[p], m[i]
        for r in range(3):
            if r != i:
                k = m[r][i] / m[i][i]
                for c in range(i, 4):
                    m[r][c] -= k * m[i][c]
    return [m[i][3] / m[i][i] for i in range(3)]


def pivot_point_x(v_lat, yaw_rate):
    """Abscisse (repère robot) du point de l'axe longitudinal qui n'a pas de
    vitesse latérale. Vitesse latérale d'un point x : v_lat + w * x = 0
    => x = -v_lat / w. Si le pivot est bien l'éperon, on doit trouver ~eperon_x.
    Renvoie None si le robot ne tourne pas."""
    if abs(yaw_rate) < 1e-3:
        return None
    return -v_lat / yaw_rate


def static_pivot_check(mass_chassis, mass_axle, axle_x, eperon_x, handle_x,
                       mu_wheel_lat, mu_eperon, g=9.81):
    """Estimation quasi-statique : le guidon fait-il pivoter le tracteur AUTOUR
    de l'éperon (comportement voulu) ou l'éperon dérape-t-il ?

    Hypothèses : châssis à x=0 (CdM), essieu à axle_x, éperon à eperon_x,
    répartition des charges par équilibre des moments en tangage.
    Retourne un dict avec la force guidon minimale pour faire glisser les roues
    latéralement et la réserve d'adhérence de l'éperon.
    """
    # Charges verticales : châssis réparti entre essieu et éperon ; essieu sur roues
    span = axle_x - eperon_x
    n_eperon = mass_chassis * g * (axle_x - 0.0) / span
    n_wheels = mass_chassis * g * (0.0 - eperon_x) / span + mass_axle * g
    f_wheels = mu_wheel_lat * n_wheels          # résistance latérale des roues
    f_eperon_max = mu_eperon * n_eperon          # adhérence latérale de l'éperon
    # Pivot autour de l'éperon : moment du guidon > moment des roues
    lever_handle = abs(handle_x - eperon_x)
    lever_wheels = abs(axle_x - eperon_x)
    f_handle_min = f_wheels * lever_wheels / lever_handle
    # Réaction latérale demandée à l'éperon pendant le pivot
    eperon_demand = f_handle_min + f_wheels
    return {
        'n_eperon': n_eperon,
        'n_wheels': n_wheels,
        'f_wheels_lat': f_wheels,
        'f_eperon_max': f_eperon_max,
        'f_handle_min': f_handle_min,
        'eperon_demand': eperon_demand,
        'pivot_ok': eperon_demand < f_eperon_max,
    }
