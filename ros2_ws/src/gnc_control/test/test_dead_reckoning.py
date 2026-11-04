"""Unit tests of gnc_control.dead_reckoning (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.dead_reckoning import DeadReckoning, GnssCorrection, interpolate_speed

FREQUENCIES = [0.0, 1.0, 2.0]
AMPLITUDES = [0.0, 30.0]
TABLE = [[0.0, 0.0], [0.0, 0.5], [0.0, 1.0]]
DT = 0.05


def robot(**kwargs):
    return DeadReckoning(FREQUENCIES, AMPLITUDES, TABLE, **kwargs)


def swim(position, seconds, frequency=2.0, amplitude=30.0, heading=0.0, pitch=0.0):
    for _ in range(int(round(seconds / DT))):
        position.update(frequency, amplitude, heading, pitch, DT)


def test_interpolation():
    assert interpolate_speed(1.0, 30.0, FREQUENCIES, AMPLITUDES, TABLE) == pytest.approx(0.5)
    assert interpolate_speed(1.5, 30.0, FREQUENCIES, AMPLITUDES, TABLE) == pytest.approx(0.75)
    assert interpolate_speed(1.0, 15.0, FREQUENCIES, AMPLITUDES, TABLE) == pytest.approx(0.25)


def test_interpolation_is_limited_to_the_table():
    assert interpolate_speed(5.0, 90.0, FREQUENCIES, AMPLITUDES, TABLE) == pytest.approx(1.0)
    assert interpolate_speed(-1.0, -5.0, FREQUENCIES, AMPLITUDES, TABLE) == pytest.approx(0.0)


def test_speed_delay():
    position = robot(time_constant_s=1.0)
    swim(position, 1.0)
    assert position.speed == pytest.approx(1.0 - math.exp(-1.0), abs=0.01)
    swim(position, 10.0)
    assert position.speed == pytest.approx(1.0, abs=0.01)


def test_no_flapping_stops_the_robot():
    position = robot()
    swim(position, 10.0)
    swim(position, 10.0, frequency=0.0, amplitude=0.0)
    assert position.speed == pytest.approx(0.0, abs=0.01)
    north = position.north
    swim(position, 5.0, frequency=0.0, amplitude=0.0)
    assert position.north == pytest.approx(north, abs=0.01)


@pytest.mark.parametrize('heading,north,west', [(0.0, 1.0, 0.0), (90.0, 0.0, -1.0), (180.0, -1.0, 0.0), (270.0, 0.0, 1.0)])
def test_direction(heading, north, west):
    position = robot(time_constant_s=1e-3)
    swim(position, 10.0, heading=heading)
    assert position.north == pytest.approx(10.0 * north, abs=0.1)
    assert position.west == pytest.approx(10.0 * west, abs=0.1)


def test_pitch_reduces_the_horizontal_speed():
    position = robot(time_constant_s=1e-3)
    swim(position, 10.0, pitch=60.0)
    assert position.north == pytest.approx(5.0, abs=0.1)


def test_speed_scale():
    position = robot(speed_scale=0.8, time_constant_s=1e-3)
    swim(position, 10.0)
    assert position.north == pytest.approx(8.0, abs=0.1)


def test_square_closes():
    position = robot(time_constant_s=1e-3)
    for heading in (0.0, 90.0, 180.0, 270.0):
        swim(position, 10.0, heading=heading)
    assert position.north == pytest.approx(0.0, abs=0.1)
    assert position.west == pytest.approx(0.0, abs=0.1)


def test_first_fix_only_sets_the_reference():
    position = robot()
    position.north, position.west = 3.0, -2.0
    GnssCorrection(0.5).apply(48.0, -4.0, position)
    assert (position.north, position.west) == (3.0, -2.0)


def test_fix_pulls_the_position():
    position = robot()
    gnss = GnssCorrection(0.5)
    gnss.apply(48.0, -4.0, position)
    gnss.apply(48.0 + math.degrees(10.0 / 6371000.0), -4.0, position)   # the receiver moved 10 m north
    assert position.north == pytest.approx(5.0, abs=0.01)
    assert position.west == pytest.approx(0.0, abs=0.01)


def test_fix_to_the_east_is_negative_west():
    position = robot()
    gnss = GnssCorrection(1.0)
    gnss.apply(48.0, -4.0, position)
    gnss.apply(48.0, -4.0 + math.degrees(10.0 / (6371000.0 * math.cos(math.radians(48.0)))), position)
    assert position.west == pytest.approx(-10.0, abs=0.01)
