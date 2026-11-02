"""Heading controller of the robot fish (pure Python, no ROS dependency, so it can be unit tested).

The robot turns by flapping its tail around a bias angle: a bias to the right makes it turn right while it
swims. The controller computes this bias from the heading error with a PI law:

    error = wrap(heading_setpoint − heading)            [deg], + = the target is to the right
    bias  = kp·error + ki·∫error dt                      [deg], + = fin to the right = turn right

limited to ±max_bias. The integral term removes a steady error (e.g. an asymmetry of the robot or a current).
Anti-windup by conditional integration: while the output is saturated, the integral only changes if this
brings the output back from the limit, so it does not grow during a large turn and cause an overshoot.

No derivative term: the heading measured on a robot fish still carries part of the stroke oscillation, which
a derivative would amplify; the damping comes from the water (the yaw rate settles by itself, see
Docs/GNC.md).
"""
from .heading import wrap_180


class HeadingPI:
    def __init__(self, kp, ki, max_bias_deg):
        self.kp = kp                                    # [deg of bias / deg of error]
        self.ki = ki                                    # [deg of bias / (deg of error · s)]
        self.max_bias_deg = max_bias_deg                # output limit [deg]
        self.integral = 0.0                             # integral term, already multiplied by ki [deg]
        self.error = 0.0                                # last error [deg] (debug)

    def reset(self):
        self.integral = 0.0
        self.error = 0.0

    def update(self, setpoint_deg, heading_deg, dt):
        """Fin bias [deg] for one control step of dt seconds."""
        self.error = wrap_180(setpoint_deg - heading_deg)
        proportional = self.kp * self.error
        candidate = self.integral + self.ki * self.error * dt          # integral if it is allowed to grow
        unclamped = proportional + candidate
        saturated = abs(unclamped) > self.max_bias_deg
        if not saturated or (unclamped > 0) != (self.error > 0):      # integrate unless it pushes further into the limit
            self.integral = candidate
        # the integral alone never exceeds the limit (it would take long to come back)
        self.integral = max(-self.max_bias_deg, min(self.max_bias_deg, self.integral))
        bias = proportional + self.integral
        return max(-self.max_bias_deg, min(self.max_bias_deg, bias))
