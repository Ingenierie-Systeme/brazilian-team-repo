"""Unit tests of gnc_control.attitude_filter and gnc_control.depth_filter (python3 -m pytest test/)."""
import math
import random

import numpy as np
import pytest

from gnc_control.attitude_filter import AttitudeFilter
from gnc_control.depth_filter import DepthFilter
from gnc_control.heading import heading_from_quaternion, roll_pitch_from_quaternion, wrap_180

RATE = 50.0                                             # [Hz]
DT = 1.0 / RATE
INCLINATION = math.radians(63.5)
EARTH_FIELD = 47.9 * np.array([math.cos(INCLINATION), 0.0, -math.sin(INCLINATION)])   # NWU [µT]
GRAVITY = np.array([0.0, 0.0, 9.81])                    # what an accelerometer at rest reads in NWU [m/s²]


def rotation(roll_deg, pitch_deg, heading_deg):
    """Body FLU → world NWU matrix. A heading clockwise from north is a yaw counterclockwise of −heading."""
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(-heading_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def still(attitude_filter, roll, pitch, heading, seconds, gyro=(0.0, 0.0, 0.0), mag_offset=(0.0, 0.0, 0.0)):
    """Feeds the filter with the readings of a robot at rest in this orientation."""
    to_body = rotation(roll, pitch, heading).T
    for _ in range(int(seconds * RATE)):
        attitude_filter.update(gyro, to_body @ GRAVITY, to_body @ EARTH_FIELD + np.array(mag_offset), DT)


@pytest.mark.parametrize('roll,pitch,heading', [(0, 0, 0), (0, 0, 90), (10, 0, 60), (0, 20, 200), (-8, 12, 359)])
def test_orientation_at_rest(roll, pitch, heading):
    f = AttitudeFilter(RATE)
    still(f, roll, pitch, heading, 5.0)
    r, p = roll_pitch_from_quaternion(*f.quaternion)
    assert math.degrees(r) == pytest.approx(roll, abs=0.1)
    assert math.degrees(p) == pytest.approx(pitch, abs=0.1)
    assert wrap_180(heading_from_quaternion(*f.quaternion) - heading) == pytest.approx(0.0, abs=0.1)


def test_not_ready_while_converging():
    f = AttitudeFilter(RATE)
    assert not f.ready
    still(f, 0, 0, 90, 1.0)
    assert not f.ready
    still(f, 0, 0, 90, 4.0)
    assert f.ready


def test_turn_is_followed():
    """A steady turn of 25°/s to the right, the fastest of the robot, from 0° to 90°."""
    f = AttitudeFilter(RATE)
    still(f, 0, 0, 0, 5.0)
    heading, rate = 0.0, 25.0
    errors = []
    while heading < 90.0:
        heading += rate * DT
        to_body = rotation(0, 0, heading).T
        f.update((0.0, 0.0, math.radians(-rate)), to_body @ GRAVITY, to_body @ EARTH_FIELD, DT)
        errors.append(abs(wrap_180(heading_from_quaternion(*f.quaternion) - heading)))
    assert max(errors) < 1.0


def test_calibration_removes_the_errors_of_the_sensors():
    offset = (20.0, -15.0, 8.0)                         # hard iron [µT]
    gyro_offset = (0.05, -0.03, 0.04)                   # [rad/s]
    f = AttitudeFilter(RATE, gyro_offset=gyro_offset, mag_hard=offset)
    still(f, 5, 10, 120, 5.0, gyro=gyro_offset, mag_offset=offset)
    assert wrap_180(heading_from_quaternion(*f.quaternion) - 120.0) == pytest.approx(0.0, abs=0.2)


def test_hard_iron_without_calibration_makes_an_error():
    f = AttitudeFilter(RATE)
    still(f, 0, 0, 120, 5.0, mag_offset=(20.0, -15.0, 8.0))
    assert abs(wrap_180(heading_from_quaternion(*f.quaternion) - 120.0)) > 5.0


def test_gyro_scale():
    """A driver reading 4 times too large, as the course code suggests for the Navigator."""
    f = AttitudeFilter(RATE, gyro_scale=0.25)
    still(f, 0, 0, 0, 5.0)
    heading, rate = 0.0, 25.0
    while heading < 90.0:
        heading += rate * DT
        to_body = rotation(0, 0, heading).T
        f.update((0.0, 0.0, 4.0 * math.radians(-rate)), to_body @ GRAVITY, to_body @ EARTH_FIELD, DT)
    assert wrap_180(heading_from_quaternion(*f.quaternion) - heading) == pytest.approx(0.0, abs=1.0)


def test_axes_of_the_sensor():
    """Sensor turned 90° about z: its x points to the left of the robot, its y to the back."""
    to_body = rotation(0, 0, 40).T
    turned = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # body → sensor
    f = AttitudeFilter(RATE, axes='NYPXPZ')             # body x = −sensor y, body y = +sensor x
    for _ in range(int(5.0 * RATE)):
        f.update((0.0, 0.0, 0.0), turned @ to_body @ GRAVITY, turned @ to_body @ EARTH_FIELD, DT)
    assert wrap_180(heading_from_quaternion(*f.quaternion) - 40.0) == pytest.approx(0.0, abs=0.2)


def test_stroke_acceleration_does_not_tilt_the_estimate():
    """Sideways acceleration of 2 m/s² at 1.5 Hz, stronger than the strokes of the tail."""
    f = AttitudeFilter(RATE)
    still(f, 0, 0, 30, 5.0)
    to_body = rotation(0, 0, 30).T
    worst = 0.0
    for i in range(int(10.0 * RATE)):
        side = 2.0 * math.sin(2.0 * math.pi * 1.5 * i * DT)
        f.update((0.0, 0.0, 0.0), to_body @ GRAVITY + np.array([0.0, side, 0.0]), to_body @ EARTH_FIELD, DT)
        roll, _ = roll_pitch_from_quaternion(*f.quaternion)
        worst = max(worst, abs(math.degrees(roll)))
    assert worst < 5.0


def test_depth_filter_first_reading_starts_it():
    f = DepthFilter()
    f.update(2.5, 0.0)
    assert f.depth == 2.5
    assert f.rate == 0.0


def test_depth_filter_follows_a_steady_descent():
    f = DepthFilter()
    depth = 1.0
    for _ in range(300):                                # 10 Hz, 30 s, going down at 0.2 m/s
        f.update(depth, 0.1)
        depth += 0.2 * 0.1
    assert f.rate == pytest.approx(0.2, abs=0.01)
    assert f.depth == pytest.approx(depth - 0.02, abs=0.02)


def test_depth_filter_reduces_the_noise():
    random.seed(1)
    f = DepthFilter()
    rates, depths = [], []
    for i in range(600):
        f.update(2.0 + random.gauss(0.0, 0.002), 0.1)   # 2 mm of noise, as the simulated sensor
        if i > 100:
            rates.append(f.rate)
            depths.append(f.depth)
    raw_rate_noise = math.sqrt(2.0) * 0.002 / 0.1       # noise of the plain difference of two readings
    assert np.std(depths) < 0.002
    assert np.std(rates) < raw_rate_noise / 2.0


def test_depth_filter_ignores_a_zero_time_step():
    f = DepthFilter()
    f.update(1.0, 0.0)
    f.update(5.0, 0.0)
    assert f.depth == 1.0
