"""Tail oscillator of the real robot fish (pure Python, no ROS dependency, so it can be unit tested).

The controllers describe the flapping with three slow commands: frequency, amplitude and bias. The real servo
only understands one thing, the angle it must reach now. This class turns the first into the second:

    phase = phase + 2π·frequency·dt
    fin   = bias + amplitude·sin(phase)                 [deg], + = fin to the right = turn right

limited to ±max_angle. It is the same law as the oscillator inside the simulator (RobotFishPhysics), so the
controllers see the same robot in the simulation and in the water. In the simulation this class is not used.

The phase is integrated, not computed as sin(2π·f·t): with sin(2π·f·t), any change of f would make the fin jump
to another point of the wave, and the jump grows with t. The time step is measured at each call, so a late
timer does not change the flapping frequency.

Safety: if no command arrives for `command_timeout_s`, the commands are forgotten and the fin goes back to the
center, as the watchdog of the simulator does. The phase restarts at 0, so the next flapping starts from the
bias and not from a peak of the wave.

Commands are normalized like the ROS topics of the simulator: frequency and amplitude in [0, 1], bias in [-1, 1].
"""
import math


def _clamp(value, low, high):
    """Limits value to [low, high]; NaN is read as 0, as in the simulator."""
    if math.isnan(value):
        return 0.0
    return max(low, min(high, value))


def fin_to_servo(fin_deg, servo_range_deg, invert=False):
    """Fin angle [deg] → servo command in [-1, 1], where ±1 is the full range of the servo (±500 µs).

    servo_range_deg: fin angle reached at +1, to be measured on the robot.
    invert: True if a positive command moves the fin to the left on the robot.
    """
    command = fin_deg / servo_range_deg
    return _clamp(-command if invert else command, -1.0, 1.0)


class TailOscillator:
    def __init__(self, max_frequency_hz, max_angle_deg, command_timeout_s):
        self.max_frequency_hz = max_frequency_hz        # frequency at command 1 [Hz]
        self.max_angle_deg = max_angle_deg              # amplitude and bias at command 1, fin limit [deg]
        self.command_timeout_s = command_timeout_s      # watchdog [s]
        self.frequency_hz = 0.0                         # current flapping frequency [Hz]
        self.amplitude_deg = 0.0                        # current flapping amplitude [deg]
        self.bias_deg = 0.0                             # current mean fin angle [deg]
        self.phase = 0.0                                # phase of the oscillation [rad]
        self.last_command_time = None                   # time of the last command [s], None = none yet
        self.last_step_time = None                      # time of the last step [s]

    def set_frequency(self, value, now):
        """value: fraction of the maximum frequency, [0, 1]. now: current time [s]."""
        self.frequency_hz = _clamp(value, 0.0, 1.0) * self.max_frequency_hz
        self.last_command_time = now

    def set_amplitude(self, value, now):
        """value: fraction of the maximum angle, [0, 1]. now: current time [s]."""
        self.amplitude_deg = _clamp(value, 0.0, 1.0) * self.max_angle_deg
        self.last_command_time = now

    def set_bias(self, value, now):
        """value: fraction of the maximum angle, [-1, 1], + = fin to the right. now: current time [s]."""
        self.bias_deg = _clamp(value, -1.0, 1.0) * self.max_angle_deg
        self.last_command_time = now

    def is_commanded(self, now):
        """True while commands keep arriving (the watchdog has not expired)."""
        return (self.last_command_time is not None
                and now - self.last_command_time <= self.command_timeout_s)

    def step(self, now):
        """Advances the oscillator to the time `now` [s] and returns the fin angle to reach [deg]."""
        dt = 0.0 if self.last_step_time is None else max(0.0, now - self.last_step_time)
        self.last_step_time = now

        if not self.is_commanded(now):                  # no command: forget everything, fin to the center
            self.frequency_hz = self.amplitude_deg = self.bias_deg = 0.0
            self.phase = 0.0
            return 0.0

        self.phase = math.fmod(self.phase + 2.0 * math.pi * self.frequency_hz * dt, 2.0 * math.pi)
        fin = self.bias_deg + self.amplitude_deg * math.sin(self.phase)
        return _clamp(fin, -self.max_angle_deg, self.max_angle_deg)
