"""Commands of the servos (pure Python, no ROS or hardware dependency, so it can be unit tested).

The servos read a pulse of 1000 to 2000 µs, repeated at 50 Hz. A command of −1 … 1 maps to this range, with 1500 µs
in the center. On the Navigator board, a pulse is set as a fraction of the 20 ms PWM period (the duty cycle).
"""
import math

PWM_FREQUENCY_HZ = 50.0
CENTER_US = 1500.0
RANGE_US = 500.0                                        # from the center to each end of the range


def command_to_duty(command, invert=False, low=-1.0, high=1.0):
    """Command in [-1, 1] → duty cycle of the PWM. NaN is read as 0.

    invert: True if a positive command must move the servo the other way.
    low, high: limits of the servo position, applied after `invert`, to keep the mechanism away from its stops. They
    are in servo units (−1 = 1000 µs, 1 = 2000 µs), so they stay right whatever the direction of the mechanism.
    """
    value = 0.0 if math.isnan(command) else max(-1.0, min(1.0, command))
    servo = max(low, min(high, -value if invert else value))
    return (CENTER_US + servo * RANGE_US) * 1e-6 * PWM_FREQUENCY_HZ


class ServoChannel:
    """One servo: applies the commands and returns to the center when they stop arriving."""

    def __init__(self, write_duty, command_timeout_s, invert=False, low=-1.0, high=1.0):
        self.write_duty = write_duty                    # function that sends a duty cycle to the board
        self.command_timeout_s = command_timeout_s      # watchdog [s]
        self.invert = invert
        self.low = low                                  # servo limits, see command_to_duty
        self.high = high
        self.last_command_time = None                   # time of the last command [s], None = at the center

    def center(self):
        """Sends the servo to the center."""
        self.write_duty(command_to_duty(0.0))

    def command(self, value, now):
        """Applies a command received at the time `now` [s]."""
        self.write_duty(command_to_duty(value, self.invert, self.low, self.high))
        self.last_command_time = now

    def check_watchdog(self, now):
        """To call regularly. Returns True when the commands have just stopped and the servo was centered."""
        if self.last_command_time is None or now - self.last_command_time <= self.command_timeout_s:
            return False
        self.center()
        self.last_command_time = None
        return True
