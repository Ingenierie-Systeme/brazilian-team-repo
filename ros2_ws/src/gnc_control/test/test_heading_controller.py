"""Unit tests of gnc_control.heading_controller (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.heading import CircularMovingAverage, wrap_180, wrap_360
from gnc_control.heading_controller import HeadingPI


def test_sign_and_shortest_turn():
    c = HeadingPI(kp=1.0, ki=0.0, max_bias_deg=30.0)
    assert c.update(10.0, 0.0, 0.05) > 0                # target to the right: fin to the right
    assert c.update(350.0, 0.0, 0.05) < 0               # 350° is 10° to the left, not 350° to the right
    assert abs(c.update(170.0, 350.0, 0.05)) == pytest.approx(30.0)   # 180° away: saturated, either side is fine


def test_saturation():
    c = HeadingPI(kp=1.0, ki=0.0, max_bias_deg=30.0)
    assert c.update(90.0, 0.0, 0.05) == pytest.approx(30.0)


def test_anti_windup():
    c = HeadingPI(kp=1.0, ki=0.5, max_bias_deg=30.0)
    for _ in range(400):                                # 20 s saturated by a 90° error
        c.update(90.0, 0.0, 0.05)
    assert c.integral == pytest.approx(0.0)             # the integral did not grow while saturated


def test_integral_removes_a_constant_error():
    c = HeadingPI(kp=0.0, ki=0.1, max_bias_deg=30.0)
    for _ in range(100):
        bias = c.update(5.0, 0.0, 0.05)
    assert bias == pytest.approx(2.5)                   # 0.1 × 5° × 5 s


def closed_loop(step_deg, gain, disturbance=0.3, duration=40.0):
    """Heading error over time for a heading step, on the yaw model identified in the simulation:
    yaw rate → gain × bias (+ a constant drift) in about 0.3 s, heading swinging ±10° at 1.5 Hz,
    measurement averaged over one flapping period, controller at 20 Hz (default gains of the node)."""
    c = HeadingPI(kp=1.5, ki=0.02, max_bias_deg=30.0)
    avg = CircularMovingAverage(1.0 / 1.5)
    psi = rate = bias = 0.0
    t, dt, next_control, errors = 0.0, 0.01, 0.0, []
    while t < duration:
        avg.add(t, wrap_360(psi + 10.0 * math.sin(2 * math.pi * 1.5 * t)))
        if t >= next_control:
            next_control += 0.05
            bias = c.update(step_deg, avg.value(), 0.05)
        rate += (gain * bias + disturbance - rate) * dt / 0.3
        psi += rate * dt
        t += dt
        errors.append((t, wrap_180(psi - step_deg)))
    return errors


@pytest.mark.parametrize('step', [30.0, 90.0, 180.0])
@pytest.mark.parametrize('gain', [0.75, 0.33])          # amplitude 30° and 15°
def test_closed_loop_step(step, gain):
    errors = closed_loop(step, gain)
    assert max(e for _, e in errors) < 6.0              # small overshoot (this simple model is pessimistic)
    assert all(abs(e) < 2.0 for t, e in errors if t > 25.0)   # settled
