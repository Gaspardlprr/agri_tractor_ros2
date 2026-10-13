import math

import pytest

from agri_tractor_ros2.handle_math import (YawRateController, fit_circle,
                                           handle_wrench_body, handle_wrench_world,
                                           pivot_point_x, quat_rotate,
                                           static_pivot_check, torque_to_handle_force,
                                           yaw_from_quat)

HANDLE = (-1.0, 0.0, 0.65)


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_push_handle_left_turns_right():
    # guidon à l'arrière poussé vers la gauche -> couple de lacet négatif (virage à droite)
    w = handle_wrench_body(100.0, HANDLE)
    assert w.force == (0.0, 100.0, 0.0)
    assert w.torque[2] == pytest.approx(-100.0)
    assert w.torque[0] == pytest.approx(-65.0)   # tendance au roulis (guidon haut)


def test_left_turn_needs_push_right():
    f = torque_to_handle_force(50.0, HANDLE[0])   # tau_z > 0 = virage à gauche
    assert f < 0


def test_world_rotation_follows_heading():
    w = handle_wrench_world(100.0, HANDLE, yaw_quat(math.pi / 2))
    # robot tourné vers +Y : sa gauche est le -X monde
    assert w.force[0] == pytest.approx(-100.0)
    assert w.force[1] == pytest.approx(0.0, abs=1e-9)
    assert w.torque[2] == pytest.approx(-100.0)   # tau_z invariant au cap


def test_quaternion_helpers():
    q = yaw_quat(0.7)
    assert yaw_from_quat(q) == pytest.approx(0.7)
    v = quat_rotate(q, (1.0, 0.0, 0.0))
    assert v[0] == pytest.approx(math.cos(0.7))
    assert v[1] == pytest.approx(math.sin(0.7))


def test_controller_closed_loop_integrates_and_antiwindup():
    c = YawRateController(k_ff=250, kp=150, ki=80, i_limit=3.0)
    tau1 = c.update(0.4, 0.0, 0.05)
    tau2 = c.update(0.4, 0.0, 0.05)
    assert tau2 > tau1 > 0
    i_before = c.integral
    c.update(0.4, 0.0, 0.05, saturated_sign=+1)   # saturé dans le même sens : gel
    assert c.integral == i_before
    c.update(0.4, 0.8, 0.05, saturated_sign=+1)   # erreur opposée : on désintègre
    assert c.integral < i_before


def test_open_loop_is_pure_feedforward():
    c = YawRateController(250, 150, 80, 3.0, closed_loop=False)
    assert c.update(0.2, 5.0, 0.05) == pytest.approx(50.0)


def test_fit_circle():
    pts = [(2 + 3.5 * math.cos(a), -1 + 3.5 * math.sin(a))
           for a in [i * 0.2 for i in range(20)]]
    cx, cy, r = fit_circle(pts)
    assert (cx, cy, r) == pytest.approx((2, -1, 3.5), abs=1e-6)
    with pytest.raises(ValueError):
        fit_circle([(0, 0), (1, 1), (2, 2)])


def test_pivot_point():
    # robot qui tourne autour de l'éperon (x=-0.45) : le CdM a v_lat = 0.45 * w
    assert pivot_point_x(0.45 * 0.5, 0.5) == pytest.approx(-0.45)
    assert pivot_point_x(0.1, 0.0) is None


def test_default_parameters_pivot_on_eperon():
    """Vérifie que les valeurs de l'URDF donnent bien un pivot sur l'éperon
    avec un effort humain (<= max_handle_force = 400 N)."""
    r = static_pivot_check(mass_chassis=90, mass_axle=20, axle_x=0.20, eperon_x=-0.45,
                           handle_x=-1.0, mu_wheel_lat=0.30, mu_eperon=3.0)
    assert r['pivot_ok'], r
    assert r['f_handle_min'] < 400
    # contre-exemple : éperon peu accrocheur -> il dérape
    bad = static_pivot_check(90, 20, 0.20, -0.45, -1.0, 0.30, 0.8)
    assert not bad['pivot_ok']
