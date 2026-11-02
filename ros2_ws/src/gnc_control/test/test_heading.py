"""Unit tests of gnc_control.heading (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.heading import (CircularMovingAverage, heading_from_quaternion, magnetic_heading,
                                 roll_pitch_from_quaternion, wrap_180, wrap_360)


def quaternion(roll, pitch, yaw):
    """Body→world quaternion (x, y, z, w) from roll, pitch, yaw [rad] (Z then Y then X)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rotate_to_body(q, v):
    """World vector v seen in the body frame (inverse rotation of q)."""
    x, y, z, w = q
    # rotation matrix of q (body → world), applied transposed
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return tuple(sum(r[i][j] * v[i] for i in range(3)) for j in range(3))


def earth_field(inclination_deg=63.5):
    """Field pointing to magnetic north (world x, NWU) and down, unit intensity."""
    inc = math.radians(inclination_deg)
    return (math.cos(inc), 0.0, -math.sin(inc))


def test_wrap():
    assert wrap_180(190) == pytest.approx(-170)
    assert wrap_180(-190) == pytest.approx(170)
    assert wrap_180(180) == pytest.approx(180)
    assert wrap_360(-10) == pytest.approx(350)
    assert wrap_360(370) == pytest.approx(10)


@pytest.mark.parametrize('heading', [0, 45, 90, 135, 180, 270, 359])
def test_level_magnetic_heading(heading):
    # a heading ψ clockwise from north is a yaw of −ψ in NWU (counterclockwise)
    q = quaternion(0.0, 0.0, math.radians(-heading))
    b = rotate_to_body(q, earth_field())
    assert wrap_180(magnetic_heading(*b) - heading) == pytest.approx(0, abs=1e-6)
    assert wrap_180(heading_from_quaternion(*q) - heading) == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize('roll,pitch', [(10, 0), (0, 15), (-8, 12), (20, -20)])
def test_tilt_compensation(roll, pitch):
    heading = 60.0
    q = quaternion(math.radians(roll), math.radians(pitch), math.radians(-heading))
    b = rotate_to_body(q, earth_field())
    r, p = roll_pitch_from_quaternion(*q)
    assert math.degrees(r) == pytest.approx(roll, abs=1e-6)
    assert math.degrees(p) == pytest.approx(pitch, abs=1e-6)
    assert wrap_180(magnetic_heading(*b, roll=r, pitch=p) - heading) == pytest.approx(0, abs=1e-6)
    # without compensation the tilt makes an error (the field points steeply down)
    assert abs(wrap_180(magnetic_heading(*b) - heading)) > 1.0


def test_moving_average_removes_the_stroke_swing():
    f = 1.5                                             # flapping frequency [Hz]
    avg = CircularMovingAverage(1.0 / f)
    t = 0.0
    while t < 3.0:                                      # heading 355° swinging ±10° with each stroke
        avg.add(t, wrap_360(355.0 + 10.0 * math.sin(2 * math.pi * f * t)))
        t += 0.05
    assert wrap_180(avg.value() - 355.0) == pytest.approx(0, abs=0.5)   # right across 0/360


def test_moving_average_empty():
    assert CircularMovingAverage(1.0).value() is None
