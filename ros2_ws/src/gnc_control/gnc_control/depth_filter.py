"""Depth and vertical speed from the pressure sensor (pure Python, no ROS dependency, so it can be unit tested).

The depth control will need the vertical speed, which no sensor measures. The difference between two noisy depth
readings would amplify the noise, so an alpha-beta filter is used, which predicts and then corrects:

    predicted = depth + rate·dt
    residual  = measured − predicted
    depth     = predicted + alpha·residual
    rate      = rate + (beta/dt)·residual

Larger alpha and beta follow changes faster, smaller values remove more noise.
"""


class DepthFilter:
    def __init__(self, alpha=0.2, beta=0.02):
        self.alpha = alpha
        self.beta = beta
        self.depth = None                               # estimated depth [m], + = below the surface
        self.rate = 0.0                                 # estimated vertical speed [m/s], + = going down

    def update(self, measured, dt):
        """One step with a new reading [m]. dt: time since the previous reading [s]."""
        if self.depth is None or dt <= 0.0:
            if self.depth is None:
                self.depth = measured                   # the first reading starts the filter
            return
        predicted = self.depth + self.rate * dt
        residual = measured - predicted
        self.depth = predicted + self.alpha * residual
        self.rate += self.beta / dt * residual
