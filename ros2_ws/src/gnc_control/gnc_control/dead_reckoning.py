"""Position of the robot by dead reckoning (pure Python, no ROS dependency, so it can be unit tested).

The robot has no speed sensor and, under water, no absolute position. Integrating the accelerometer twice diverges
within seconds. So the speed is predicted from the flapping commands, with a table measured in the simulation, and
integrated along the estimated heading:

    speed     → table(frequency, amplitude), with a first-order delay
    north    += speed · cos(pitch) · cos(heading) · dt
    west     −= speed · cos(pitch) · sin(heading) · dt

The error grows with the distance traveled: this is an estimate, not a measurement. The table must be calibrated on
the real robot (`speed_scale`). It ignores the speed lost in turns, and any water current.

If a GNSS receiver is available at the surface, each valid fix pulls the estimate toward it with a small gain,
because a fix has an error of a few meters.

Frame: origin at the start, x north, y west, z up (NWU, as the orientation).
"""
import bisect
import math

EARTH_RADIUS = 6371000.0                                # [m], enough over the size of a basin


def interpolate_speed(frequency_hz, amplitude_deg, frequencies, amplitudes, table):
    """Bilinear interpolation of a speed table, limited to its range.

    table[i][j] is the speed [m/s] at frequencies[i] and amplitudes[j]; both lists are increasing.
    """
    def locate(value, grid):
        value = max(grid[0], min(grid[-1], value))
        i = min(bisect.bisect_right(grid, value) - 1, len(grid) - 2)
        return i, (value - grid[i]) / (grid[i + 1] - grid[i])

    i, u = locate(frequency_hz, frequencies)
    j, v = locate(amplitude_deg, amplitudes)
    return ((1 - u) * (1 - v) * table[i][j] + u * (1 - v) * table[i + 1][j]
            + (1 - u) * v * table[i][j + 1] + u * v * table[i + 1][j + 1])


class DeadReckoning:
    def __init__(self, frequencies, amplitudes, table, speed_scale=1.0, time_constant_s=1.2):
        self.frequencies = frequencies                  # [Hz]
        self.amplitudes = amplitudes                    # [deg]
        self.table = table                              # table[frequency][amplitude], [m/s]
        self.speed_scale = speed_scale                  # calibration on the real robot
        self.time_constant_s = time_constant_s          # delay of the speed after a command [s]
        self.speed = 0.0                                # estimated speed [m/s]
        self.north = 0.0                                # [m]
        self.west = 0.0                                 # [m]

    def update(self, frequency_hz, amplitude_deg, heading_deg, pitch_deg, dt):
        """One step. Frequency and amplitude are the current flapping commands, 0 if the robot does not flap."""
        target = self.speed_scale * interpolate_speed(frequency_hz, amplitude_deg, self.frequencies, self.amplitudes,
                                                      self.table)
        self.speed += (target - self.speed) * (1.0 - math.exp(-dt / self.time_constant_s))
        horizontal = self.speed * math.cos(math.radians(pitch_deg))
        heading = math.radians(heading_deg)
        self.north += horizontal * math.cos(heading) * dt
        self.west -= horizontal * math.sin(heading) * dt


class GnssCorrection:
    """Converts valid fixes to the local frame and pulls the position toward them."""

    def __init__(self, gain):
        self.gain = gain                                # fraction of the difference corrected at each fix
        self.reference = None                           # (latitude, longitude, north, west) of the first fix

    def apply(self, latitude, longitude, position):
        """position: DeadReckoning. The first fix only sets the reference, since the origin is the start point."""
        if self.reference is None:
            self.reference = (latitude, longitude, position.north, position.west)
            return
        lat0, lon0, north0, west0 = self.reference
        north = north0 + math.radians(latitude - lat0) * EARTH_RADIUS
        west = west0 - math.radians(longitude - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
        position.north += self.gain * (north - position.north)
        position.west += self.gain * (west - position.west)
