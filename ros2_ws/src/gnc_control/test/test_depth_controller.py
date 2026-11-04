"""Unit tests of gnc_control.depth_controller (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.depth_controller import DepthController


def controller(**kwargs):
    gains = dict(kp=30.0, ki=2.0, kd=30.0, max_pitch_deg=20.0, kp_pitch=0.01, ki_pitch=0.002, kq=0.005,
                 total_mass=4.713, moving_mass=0.713, bg=0.010, travel=0.075)
    gains.update(kwargs)
    return DepthController(**gains)


def test_target_deeper_moves_the_mass_forward():
    assert controller().update(3.0, 2.0, 0.0, 0.0, 0.0, True, 0.05) > 0.0


def test_target_shallower_moves_the_mass_back():
    assert controller().update(1.0, 2.0, 0.0, 0.0, 0.0, True, 0.05) < 0.0


def test_pitch_setpoint_is_limited():
    c = controller()
    c.update(4.0, 2.0, 0.0, 0.0, 0.0, True, 0.05)
    assert c.pitch_wanted == pytest.approx(20.0)


def test_feedforward_is_the_static_balance():
    # tan(pitch) = m·Δx / (M·BG), Δx = command × travel
    c = controller()
    command = c.feedforward(12.6)
    assert math.degrees(math.atan(0.713 * command * 0.075 / (4.713 * 0.010))) == pytest.approx(12.6)
    assert command == pytest.approx(0.2, abs=0.01)                      # measured in the simulation


def test_command_is_limited():
    c = controller(travel=0.005)
    assert c.update(4.0, 2.0, 0.0, 0.0, 0.0, True, 0.05) == 1.0


def test_no_swimming_no_pitch_no_integral():
    c = controller()
    for _ in range(200):
        c.update(4.0, 2.0, 0.0, 0.0, 0.0, False, 0.05)
    assert c.pitch_wanted == 0.0
    assert c.depth_integral == 0.0
    assert c.pitch_integral == 0.0


def test_no_windup_while_saturated():
    c = controller()
    for _ in range(400):
        c.update(4.0, 2.0, 0.0, 0.0, 0.0, True, 0.05)                  # 2 m of error: pitch saturated
    assert c.depth_integral == pytest.approx(0.0)


def test_integral_grows_with_a_small_error():
    c = controller()
    for _ in range(100):
        c.update(2.1, 2.0, 0.0, 0.0, 0.0, True, 0.05)
    assert c.depth_integral == pytest.approx(1.0)                       # 2 × 0.1 m × 5 s


def test_pitch_rate_damps():
    c = controller()
    assert c.update(2.0, 2.0, 0.0, 0.0, 10.0, True, 0.05) < 0.0        # nose going down: mass back


def closed_loop(step, speed, heavy=0.0, duration=70.0):
    """Depth error over time, on the plant identified in the simulation: pitch follows the moving mass with a delay,
    63° per unit of command, the mass moves at 0.05/0.075 per second, vertical speed = speed·sin(pitch) + drift."""
    c = controller()
    depth, rate, pitch, mass, t, dt, errors = 2.0, 0.0, 0.0, 0.0, 0.0, 0.05, []
    while t < duration:
        command = c.update(2.0 + step, depth, rate, pitch, 0.0, True, dt)
        mass += max(-0.667 * dt, min(0.667 * dt, command - mass))
        pitch += (math.degrees(math.atan(0.713 * mass * 0.075 / (4.713 * 0.010))) - pitch) * dt / 1.5
        rate = speed * math.sin(math.radians(pitch)) + heavy
        depth += rate * dt
        t += dt
        errors.append((t, 2.0 + step - depth))
    return errors


@pytest.mark.parametrize('step', [1.0, -1.0])
@pytest.mark.parametrize('speed', [0.36, 0.69])
@pytest.mark.parametrize('heavy', [0.0, 0.02, -0.02])
def test_closed_loop_step(step, speed, heavy):
    errors = closed_loop(step, speed, heavy)
    assert all(abs(e) < 0.1 for t, e in errors if t > 50.0)            # settled, also with a drift
