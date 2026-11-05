"""Keyboard teleoperation of the robot fish: the keys and what they do (pure Python, so it can be unit tested).

Any control key takes the robot: from then on, the commands of this operator replace those of the controllers, and a
running mission ends. The same keys as in Unity:

    Space   swim / stop swimming          A / D   tail bias to the left / right
    W / S   moving mass forward / back    C       center the bias and the moving mass
    X       stop: no flapping, centered   R       release the robot
"""


class TeleopState:
    def __init__(self, frequency_hz, amplitude_deg, max_frequency_hz, max_angle_deg, bias_step_deg=5.0, mass_step=0.1):
        self.frequency_hz = frequency_hz                # flapping while swimming [Hz]
        self.amplitude_deg = amplitude_deg              # flapping while swimming [deg]
        self.max_frequency_hz = max_frequency_hz        # scale of the commands [Hz]
        self.max_angle_deg = max_angle_deg              # scale of the commands [deg]
        self.bias_step_deg = bias_step_deg              # [deg per key press]
        self.mass_step = mass_step                      # [command per key press]
        self.active = False                             # the operator has the robot
        self.swimming = False
        self.bias_deg = 0.0
        self.mass = 0.0                                 # moving-mass command, + = forward

    def key(self, key):
        """Applies a key. Returns False for a key that does nothing."""
        key = key.lower()
        if key == 'r':
            self.active = self.swimming = False
            self.bias_deg = self.mass = 0.0
            return True
        if key == ' ':
            self.swimming = not self.swimming
        elif key == 'a':
            self.bias_deg = max(-self.max_angle_deg / 2.0, self.bias_deg - self.bias_step_deg)
        elif key == 'd':
            self.bias_deg = min(self.max_angle_deg / 2.0, self.bias_deg + self.bias_step_deg)
        elif key == 'w':
            self.mass = min(1.0, self.mass + self.mass_step)
        elif key == 's':
            self.mass = max(-1.0, self.mass - self.mass_step)
        elif key == 'c':
            self.bias_deg = self.mass = 0.0
        elif key == 'x':
            self.swimming = False
            self.bias_deg = self.mass = 0.0
        else:
            return False
        self.active = True
        return True

    def commands(self):
        """(frequency, amplitude, bias, moving mass), normalized like the ROS topics."""
        frequency = self.frequency_hz / self.max_frequency_hz if self.swimming else 0.0
        amplitude = self.amplitude_deg / self.max_angle_deg if self.swimming else 0.0
        return frequency, amplitude, self.bias_deg / self.max_angle_deg, self.mass

    def describe(self):
        if not self.active:
            return 'released: autonomous control, any control key takes the robot'
        return (f'TELEOP swimming={self.swimming} bias={self.bias_deg:+.0f} deg moving mass={self.mass:+.1f}')
