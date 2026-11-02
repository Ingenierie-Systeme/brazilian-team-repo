"""Heading of the robot from its sensors (pure functions, no ROS dependency, so they can be unit tested).

Conventions (see Docs/AUV_Sensors.md):
  - body frame of the sensors: ROS FLU (x forward, y left, z up);
  - world frame of the IMU orientation: x north, y west, z up (NWU);
  - heading: degrees clockwise from true north, in [0, 360) (0 = north, 90 = east), like a compass.
"""
import math
from collections import deque


def wrap_180(angle_deg):
    """Angle in (-180, 180] degrees (e.g. a heading error: + = the target is to the right)."""
    a = math.fmod(angle_deg, 360.0)                     # (-360, 360)
    if a > 180.0:
        a -= 360.0
    elif a <= -180.0:
        a += 360.0
    return a


def wrap_360(angle_deg):
    """Angle in [0, 360) degrees (a heading)."""
    a = math.fmod(angle_deg, 360.0)
    return a + 360.0 if a < 0.0 else a


def roll_pitch_from_quaternion(x, y, z, w):
    """Roll and pitch [rad] of a body→world quaternion (yaw-pitch-roll convention, Z then Y then X)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))   # clamped against rounding beyond ±1
    return roll, math.asin(sin_pitch)


def heading_from_quaternion(x, y, z, w):
    """Heading [deg, clockwise from north] of a body→world (NWU) quaternion. Used only to evaluate the
    controller: the simulated IMU orientation is perfect, a real robot does not have it."""
    yaw_nwu = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))   # counterclockwise from north
    return wrap_360(-math.degrees(yaw_nwu))


def magnetic_heading(bx, by, bz, roll=0.0, pitch=0.0):
    """Tilt-compensated magnetic heading [deg, clockwise from magnetic north] from the magnetometer reading
    (body frame FLU, any unit) and the roll and pitch of the robot [rad].

    The reading is rotated back to the level frame (undo the roll, then the pitch); in that frame the
    horizontal field points to magnetic north, and for a robot turned right by ψ, north appears ψ to the left:
    (h_x, h_y) = H·(cos ψ, sin ψ), so ψ = atan2(h_y, h_x).
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    vy = by * cr - bz * sr                              # undo the roll (rotation about x)
    vz = by * sr + bz * cr
    hx = bx * cp + vz * sp                              # undo the pitch (rotation about y)
    hy = vy
    return wrap_360(math.degrees(math.atan2(hy, hx)))


class CircularMovingAverage:
    """Average of an angle over a sliding time window.

    The heading of the robot fish swings with every stroke of the tail; averaging it over exactly one
    flapping period removes this oscillation. Angles are averaged as unit vectors (sum of sin and cos), so
    the average stays right across 0/360°.
    """

    def __init__(self, window_s):
        self.window_s = window_s                        # length of the window [s]
        self.samples = deque()                          # (time [s], sin, cos)
        self.sum_sin = 0.0
        self.sum_cos = 0.0

    def add(self, t, angle_deg):
        """Adds a sample taken at time t [s] and drops the samples older than the window."""
        a = math.radians(angle_deg)
        s, c = math.sin(a), math.cos(a)
        self.samples.append((t, s, c))
        self.sum_sin += s
        self.sum_cos += c
        while self.samples and self.samples[0][0] < t - self.window_s:
            _, s0, c0 = self.samples.popleft()
            self.sum_sin -= s0
            self.sum_cos -= c0

    def value(self):
        """Average angle [deg, 0–360), or None before the first sample."""
        if not self.samples:
            return None
        return wrap_360(math.degrees(math.atan2(self.sum_sin, self.sum_cos)))

    def reset(self):
        self.samples.clear()
        self.sum_sin = self.sum_cos = 0.0
