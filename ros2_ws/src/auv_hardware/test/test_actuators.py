"""Unit tests of auv_hardware.actuators (no ROS or hardware needed: python3 -m pytest test/)."""
import pytest

from auv_hardware.actuators import ServoChannel, command_to_duty

TIMEOUT = 0.5
CENTER = 0.075                                          # 1500 µs of the 20 ms period
FULL = 0.1                                              # 2000 µs
LOW = 0.05                                              # 1000 µs


def channel(**kwargs):
    written = []
    return ServoChannel(written.append, TIMEOUT, **kwargs), written


@pytest.mark.parametrize('command,duty', [(0.0, CENTER), (1.0, FULL), (-1.0, LOW), (0.5, 0.0875)])
def test_duty(command, duty):
    assert command_to_duty(command) == pytest.approx(duty)


@pytest.mark.parametrize('command,duty', [(1.01, FULL), (5.0, FULL), (1500.0, FULL), (-1.2, LOW)])
def test_out_of_range_never_goes_to_the_other_side(command, duty):
    assert command_to_duty(command) == pytest.approx(duty)


def test_nan_is_the_center():
    assert command_to_duty(float('nan')) == pytest.approx(CENTER)


def test_invert_and_limit():
    assert command_to_duty(1.0, invert=True) == pytest.approx(LOW)
    assert command_to_duty(1.0, limit=0.5) == pytest.approx(0.0875)
    assert command_to_duty(-1.0, limit=0.5) == pytest.approx(0.0625)


def test_watchdog_centers_once():
    servo, written = channel()
    servo.command(1.0, 10.0)
    assert written[-1] == pytest.approx(FULL)
    assert not servo.check_watchdog(10.0 + TIMEOUT)     # still in time
    assert servo.check_watchdog(10.0 + TIMEOUT + 0.01)  # the commands stopped
    assert written[-1] == pytest.approx(CENTER)
    count = len(written)
    assert not servo.check_watchdog(20.0)               # already centered: nothing more to send
    assert len(written) == count


def test_no_watchdog_before_the_first_command():
    servo, written = channel()
    assert not servo.check_watchdog(100.0)
    assert written == []


def test_commands_after_the_watchdog_work_again():
    servo, written = channel()
    servo.command(1.0, 0.0)
    servo.check_watchdog(1.0)
    servo.command(-1.0, 2.0)
    assert written[-1] == pytest.approx(LOW)
