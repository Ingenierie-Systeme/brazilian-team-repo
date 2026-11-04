"""Attitude estimation from the gyroscope, accelerometer and magnetometer (no ROS dependency, can be unit tested).

The robot only has raw sensors, so its orientation must be estimated:
  - the gyroscope follows fast rotations, but its integral drifts;
  - the accelerometer gives the direction of gravity (roll and pitch), but also feels the strokes of the tail;
  - the magnetometer gives the direction of magnetic north (heading), but can be disturbed.

The AHRS of the `imufusion` library, used by the node_fusion.py given in the course, combines the three. It also
ignores the accelerometer or the magnetometer for a while when they disagree too much with the gyroscope.

Conventions (see Docs/AUV_Sensors.md):
  - body frame: ROS FLU (x forward, y left, z up);
  - world frame: x magnetic north, y west, z up (NWU). The declination is applied later, by the navigation node;
  - quaternion: body → world.

Calibration, applied in this order to each reading [in the units of the driver]:
    gyro  = gyro_scale · (gyro − gyro_offset)
    accel = accel_matrix · (accel − accel_offset)
    mag   = mag_soft · (mag − mag_hard)                  [µT]
The values come from the imu_calib tools of the course. The neutral values fit the simulation.
"""
import numpy as np
import imufusion

GRAVITY = 9.81                                          # scale of the accelerometer calibration [m/s²]


class AttitudeFilter:
    def __init__(self, sample_rate_hz, gain=0.5, acceleration_rejection_deg=10.0, magnetic_rejection_deg=10.0,
                 rejection_timeout_s=5.0, axes='PXPYPZ', gyro_scale=1.0, gyro_offset=(0.0, 0.0, 0.0),
                 accel_offset=(0.0, 0.0, 0.0), accel_matrix=None, mag_hard=(0.0, 0.0, 0.0), mag_soft=None):
        """axes: name of an imufusion alignment, from the axes of the sensor to FLU (PXPYPZ = already FLU)."""
        self.alignment = getattr(imufusion, 'ALIGNMENT_' + axes)
        self.gyro_scale = gyro_scale
        self.gyro_offset = np.array(gyro_offset, dtype=float)
        self.accel_offset = np.array(accel_offset, dtype=float)
        self.accel_matrix = np.eye(3) if accel_matrix is None else np.array(accel_matrix, dtype=float).reshape(3, 3)
        self.mag_hard = np.array(mag_hard, dtype=float)
        self.mag_soft = np.eye(3) if mag_soft is None else np.array(mag_soft, dtype=float).reshape(3, 3)

        self.ahrs = imufusion.Ahrs()
        self.ahrs.set_settings(imufusion.AhrsSettings(
            sample_rate=int(sample_rate_hz),
            convention=imufusion.CONVENTION_NWU,
            gain=gain,                                  # weight of the accelerometer and magnetometer
            gyroscope_range=2000,                       # [deg/s]
            acceleration_rejection=acceleration_rejection_deg,
            magnetic_rejection=magnetic_rejection_deg,
            rejection_timeout=int(rejection_timeout_s * sample_rate_hz)))   # [samples]
        self.bias = imufusion.Bias()                    # removes the gyroscope offset while the robot is still
        self.bias.set_settings(imufusion.BiasSettings(sample_rate=int(sample_rate_hz)))

    def update(self, gyro_rad_s, accel_m_s2, mag_ut, dt):
        """One step with the latest readings. dt: time since the previous step [s]."""
        gyro = self.gyro_scale * (np.asarray(gyro_rad_s, dtype=float) - self.gyro_offset)
        accel = self.accel_matrix @ (np.asarray(accel_m_s2, dtype=float) - self.accel_offset)
        mag = self.mag_soft @ (np.asarray(mag_ut, dtype=float) - self.mag_hard)

        gyro_deg_s = self.bias.update(imufusion.remap(np.degrees(gyro), self.alignment))
        self.ahrs.set_sample_period(dt)
        self.ahrs.update(gyro_deg_s, imufusion.remap(accel / GRAVITY, self.alignment),
                         imufusion.remap(mag, self.alignment))

    @property
    def ready(self):
        """False during the first seconds, while the filter converges: the estimate must not be used yet."""
        return not self.ahrs.get_flags().startup

    @property
    def quaternion(self):
        """Body → world quaternion (x, y, z, w), the order of ROS messages."""
        w, x, y, z = self.ahrs.get_quaternion()
        return float(x), float(y), float(z), float(w)
