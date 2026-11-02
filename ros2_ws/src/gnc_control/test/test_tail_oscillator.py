"""Unit tests of gnc_control.tail_oscillator (no ROS needed: python3 -m pytest test/)."""
import math

import pytest

from gnc_control.tail_oscillator import TailOscillator, fin_to_servo

MAX_FREQUENCY = 3.0                                     # [Hz], as in the simulator
MAX_ANGLE = 60.0                                        # [deg], as in the simulator
TIMEOUT = 0.5                                           # [s]
DT = 0.02                                               # 50 Hz


def oscillator():
    return TailOscillator(MAX_FREQUENCY, MAX_ANGLE, TIMEOUT)


def command(osc, now, frequency_hz, amplitude_deg, bias_deg=0.0):
    """Sends the three commands in physical units."""
    osc.set_frequency(frequency_hz / MAX_FREQUENCY, now)
    osc.set_amplitude(amplitude_deg / MAX_ANGLE, now)
    osc.set_bias(bias_deg / MAX_ANGLE, now)


def run(osc, start, duration, frequency_hz, amplitude_deg, bias_deg=0.0, dt=DT):
    """Runs with commands refreshed at each step; returns the list of (time, fin angle)."""
    samples = []
    steps = int(round(duration / dt))
    for i in range(steps):
        t = start + i * dt
        command(osc, t, frequency_hz, amplitude_deg, bias_deg)
        samples.append((t, osc.step(t)))
    return samples


def upward_zero_crossings(samples, level=0.0):
    return sum(1 for (_, a), (_, b) in zip(samples, samples[1:]) if a < level <= b)


def test_center_before_any_command():
    osc = oscillator()
    assert osc.step(0.0) == 0.0
    assert osc.step(10.0) == 0.0
    assert not osc.is_commanded(10.0)


def test_waveform():
    fins = [fin for _, fin in run(oscillator(), 0.0, 10.0, 1.5, 30.0)]
    assert max(fins) == pytest.approx(30.0, abs=0.2)
    assert min(fins) == pytest.approx(-30.0, abs=0.2)
    assert sum(fins) / len(fins) == pytest.approx(0.0, abs=0.5)


def test_frequency():
    samples = run(oscillator(), 0.0, 10.0, 1.5, 30.0)
    assert upward_zero_crossings(samples) == pytest.approx(15, abs=1)   # 1.5 Hz during 10 s


def test_bias_moves_the_center_of_the_wave():
    fins = [fin for _, fin in run(oscillator(), 0.0, 10.0, 1.5, 20.0, bias_deg=-15.0)]
    assert max(fins) == pytest.approx(5.0, abs=0.2)
    assert min(fins) == pytest.approx(-35.0, abs=0.2)


def test_limit_of_the_fin():
    fins = [fin for _, fin in run(oscillator(), 0.0, 5.0, 1.5, 50.0, bias_deg=30.0)]
    assert max(fins) == pytest.approx(MAX_ANGLE)        # 30 + 50 would be 80°
    assert min(fins) == pytest.approx(-20.0, abs=0.2)


def test_no_jump_when_the_frequency_changes():
    osc = oscillator()
    run(osc, 0.0, 10.0, 1.0, 30.0)                      # long enough for sin(2π·f·t) to jump a lot
    before = osc.step(9.99)
    command(osc, 10.0, 2.0, 30.0)
    after = osc.step(10.0)
    largest_step = 2.0 * math.pi * 2.0 * 30.0 * DT      # steepest slope of the wave × one time step
    assert abs(after - before) <= largest_step


def test_late_timer_keeps_the_frequency():
    osc = oscillator()
    t, samples = 0.0, []
    delays = [0.01, 0.03, 0.02, 0.05, 0.015, 0.025]     # irregular timer
    for i in range(600):
        command(osc, t, 1.5, 30.0)
        samples.append((t, osc.step(t)))
        t += delays[i % len(delays)]
    assert upward_zero_crossings(samples) == pytest.approx(1.5 * t, abs=1)


@pytest.mark.parametrize('value', [-0.5, 0.0])
def test_negative_frequency_and_amplitude_do_not_flap(value):
    osc = oscillator()
    osc.set_frequency(value, 0.0)
    osc.set_amplitude(value, 0.0)
    assert osc.frequency_hz == 0.0
    assert osc.amplitude_deg == 0.0


def test_values_out_of_range_are_limited():
    osc = oscillator()
    osc.set_frequency(5.0, 0.0)
    osc.set_amplitude(5.0, 0.0)
    osc.set_bias(-5.0, 0.0)
    assert osc.frequency_hz == MAX_FREQUENCY
    assert osc.amplitude_deg == MAX_ANGLE
    assert osc.bias_deg == -MAX_ANGLE


def test_nan_is_read_as_zero():
    osc = oscillator()
    command(osc, 0.0, 1.5, 30.0, 10.0)
    osc.set_frequency(float('nan'), 0.0)
    osc.set_amplitude(float('nan'), 0.0)
    osc.set_bias(float('nan'), 0.0)
    assert osc.frequency_hz == 0.0
    assert osc.amplitude_deg == 0.0
    assert osc.bias_deg == 0.0
    assert osc.step(0.02) == 0.0


def test_watchdog_centers_the_fin():
    osc = oscillator()
    run(osc, 0.0, 2.0, 1.5, 30.0, bias_deg=10.0)        # last command at t = 1.98 s
    assert osc.is_commanded(1.98 + TIMEOUT)
    assert not osc.is_commanded(1.98 + TIMEOUT + DT)
    assert osc.step(1.98 + TIMEOUT + DT) == 0.0
    assert osc.bias_deg == 0.0                          # the old commands are forgotten


def test_restart_after_the_watchdog_starts_from_the_bias():
    osc = oscillator()
    run(osc, 0.0, 2.0, 1.5, 30.0)
    osc.step(5.0)                                       # watchdog expired
    command(osc, 6.0, 1.5, 30.0, 10.0)
    assert osc.step(6.0) == pytest.approx(10.0, abs=4.0)   # near the bias, not at a peak of the wave


def test_one_command_is_enough_to_keep_it_alive():
    osc = oscillator()
    command(osc, 0.0, 1.5, 30.0)
    for i in range(1, 100):                             # only the bias keeps arriving, as in the simulator
        t = i * DT
        osc.set_bias(0.0, t)
        osc.step(t)
    assert osc.amplitude_deg == 30.0


def test_fin_to_servo():
    assert fin_to_servo(30.0, 60.0) == pytest.approx(0.5)
    assert fin_to_servo(-30.0, 60.0) == pytest.approx(-0.5)
    assert fin_to_servo(30.0, 60.0, invert=True) == pytest.approx(-0.5)


def test_fin_to_servo_never_leaves_the_range():
    assert fin_to_servo(60.0, 45.0) == 1.0              # the servo reaches less than the commanded angle
    assert fin_to_servo(-60.0, 45.0) == -1.0
