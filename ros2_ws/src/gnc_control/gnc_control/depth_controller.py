"""Depth controller of the robot fish (pure Python, no ROS dependency, so it can be unit tested).

The robot has no ballast: it changes depth by swimming with its nose tilted by the moving mass, so the vertical speed
is v·sin(pitch). The controller is a cascade:

  1. depth → pitch:
         error       = depth setpoint − depth              [m], + = the target is deeper
         pitch_wanted = kp·error + ∫ki·error dt − kd·rate  [deg], + = nose down, limited to ±max_pitch
     The integral removes the steady error of a robot that is not exactly neutral.

  2. pitch → moving mass:
         command = feedforward(pitch_wanted) + kp_pitch·e + ∫ki_pitch·e dt − kq·pitch rate   in [-1, 1], + = forward
     where e = pitch_wanted − pitch. The feedforward is the static balance: moving the mass m by Δx tilts the robot
     until tan(pitch) = m·Δx / (M·BG), so Δx = M·BG·tan(pitch) / m. The rest corrects errors in BG and the nose-up
     moment of the thrust; the integral acts slowly.

The pitch only changes the depth while the robot swims. When it does not swim, the pitch setpoint is 0 and the
integrals do not change, since they could only grow. Anti-windup: an integral does not grow while its output is limited.
"""
import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


class DepthController:
    def __init__(self, kp, ki, kd, max_pitch_deg, kp_pitch, ki_pitch, kq, total_mass, moving_mass, bg, travel):
        self.kp = kp                                    # [deg of pitch / m]
        self.ki = ki                                    # [deg of pitch / (m·s)]
        self.kd = kd                                    # [deg of pitch / (m/s)]
        self.max_pitch_deg = max_pitch_deg              # limit of the pitch setpoint [deg]
        self.kp_pitch = kp_pitch                        # [command / deg]
        self.ki_pitch = ki_pitch                        # [command / (deg·s)]
        self.kq = kq                                    # [command / (deg/s)]
        self.total_mass = total_mass                    # [kg]
        self.moving_mass = moving_mass                  # [kg]
        self.bg = bg                                    # distance from the center of buoyancy to the center of mass [m]
        self.travel = travel                            # moving-mass travel from the rest position at command 1 [m]
        self.depth_integral = 0.0                       # already multiplied by ki [deg]
        self.pitch_integral = 0.0                       # already multiplied by ki_pitch [command]
        self.error = 0.0                                # last depth error [m] (debug)
        self.pitch_wanted = 0.0                         # last pitch setpoint [deg] (debug)

    def feedforward(self, pitch_deg):
        """Moving-mass command that holds this pitch at rest (static balance)."""
        offset = self.total_mass * self.bg * math.tan(math.radians(pitch_deg)) / self.moving_mass
        return offset / self.travel

    def update(self, setpoint, depth, depth_rate, pitch_deg, pitch_rate_deg_s, swimming, dt):
        """One step; returns the moving-mass command in [-1, 1]. Depths [m], rates per second."""
        self.error = setpoint - depth
        if swimming:
            raw = self.kp * self.error + self.depth_integral - self.kd * depth_rate
            pitch_wanted = clamp(raw, self.max_pitch_deg)
            if raw == pitch_wanted or raw * self.error < 0.0:           # not saturated, or moving away from the limit
                self.depth_integral += self.ki * self.error * dt
        else:
            pitch_wanted = 0.0
        self.pitch_wanted = pitch_wanted

        pitch_error = pitch_wanted - pitch_deg
        raw = (self.feedforward(pitch_wanted) + self.kp_pitch * pitch_error + self.pitch_integral
               - self.kq * pitch_rate_deg_s)
        command = clamp(raw, 1.0)
        if swimming and (raw == command or raw * pitch_error < 0.0):
            self.pitch_integral += self.ki_pitch * pitch_error * dt
        return command
