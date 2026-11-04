"""Unit tests of gnc_control.heading (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.heading import (CircularMovingAverage, heading_from_quaternion, roll_pitch_from_quaternion,
                                 wrap_180, wrap_360)


def quaternion(roll, pitch, yaw):
    """Body→world quaternion (x, y, z, w) from roll, pitch, yaw [rad] (Z then Y then X)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def test_wrap():
    assert wrap_180(190) == pytest.approx(-170)
    assert wrap_180(-190) == pytest.approx(170)
    assert wrap_180(180) == pytest.approx(180)
    assert wrap_360(-10) == pytest.approx(350)
    assert wrap_360(370) == pytest.approx(10)


@pytest.mark.parametrize('heading', [0, 45, 90, 135, 180, 270, 359])
def test_heading_from_quaternion(heading):
    # a heading ψ clockwise from north is a yaw of −ψ in NWU (counterclockwise)
    q = quaternion(0.0, 0.0, math.radians(-heading))
    assert wrap_180(heading_from_quaternion(*q) - heading) == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize('roll,pitch', [(10, 0), (0, 15), (-8, 12), (20, -20)])
def test_roll_pitch_from_quaternion(roll, pitch):
    q = quaternion(math.radians(roll), math.radians(pitch), math.radians(-60.0))
    r, p = roll_pitch_from_quaternion(*q)
    assert math.degrees(r) == pytest.approx(roll, abs=1e-6)
    assert math.degrees(p) == pytest.approx(pitch, abs=1e-6)
    assert wrap_180(heading_from_quaternion(*q) - 60.0) == pytest.approx(0, abs=1e-6)


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
