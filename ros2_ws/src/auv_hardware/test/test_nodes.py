"""Tests of the driver nodes with fake hardware (needs ROS 2, no board: python3 -m pytest test/)."""
import sys
import types

import pytest

pytest.importorskip('rclpy')
import rclpy  # noqa: E402

fake = types.ModuleType('bluerobotics_navigator')   # replaces the library of the board
fake.calls = []
fake.values = types.SimpleNamespace(x=1.0, y=2.0, z=3.0)
fake.NavigatorVersion = types.SimpleNamespace(Version2='v2')
fake.Raspberry = types.SimpleNamespace(Pi4='pi4')
for name in ('set_rgb_led_strip_size', 'set_navigator_version', 'set_raspberry_pi_version', 'init', 'set_pwm_freq_hz',
             'set_pwm_enable', 'set_pwm_channel_duty_cycle'):
    setattr(fake, name, lambda *args, name=name: fake.calls.append((name, *args)))
fake.read_accel = fake.read_gyro = fake.read_mag = lambda: fake.values
sys.modules['bluerobotics_navigator'] = fake

from auv_hardware.barometer_node import BarometerNode, depth_from_pressure  # noqa: E402
from auv_hardware.navigator_node import NavigatorNode  # noqa: E402


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeSensor:
    def __init__(self, found=True):
        self.found = found
        self.pressure_pa = 101000.0

    def init(self):
        return self.found

    def read(self):
        return True

    def pressure(self, unit):
        assert unit == 100.0                            # Pa
        return self.pressure_pa


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def navigator_node():
    fake.calls.clear()
    node = NavigatorNode()
    yield node
    node.destroy_node()


def test_servos_centered_before_enabling(navigator_node):
    names = [call[0] for call in fake.calls]
    enable = names.index('set_pwm_enable')
    centered = [call for call in fake.calls[:enable] if call[0] == 'set_pwm_channel_duty_cycle']
    assert sorted(call[1] for call in centered) == [0, 1]
    assert all(call[2] == pytest.approx(0.075) for call in centered)
    assert fake.calls[enable] == ('set_pwm_enable', True)


def test_magnetometer_axes_and_units(navigator_node):
    navigator_node.pub_mag = Capture()
    navigator_node.read_mag()
    field = navigator_node.pub_mag.messages[0].magnetic_field
    assert (field.x, field.y, field.z) == pytest.approx((-2e-6, 1e-6, 3e-6))


def test_imu_has_no_orientation(navigator_node):
    navigator_node.pub_imu = Capture()
    navigator_node.read_imu()
    message = navigator_node.pub_imu.messages[0]
    assert message.orientation_covariance[0] == -1.0
    assert message.linear_acceleration.y == 2.0
    assert message.angular_velocity.z == 3.0


def test_failed_reading_does_not_stop_the_node(navigator_node):
    def fail():
        raise RuntimeError('bus')

    original = fake.read_mag
    fake.read_mag = fail
    try:
        navigator_node.read_mag()
    finally:
        fake.read_mag = original


def test_stop_centers_and_disables(navigator_node):
    fake.calls.clear()
    navigator_node.stop_servos()
    assert all(call[2] == pytest.approx(0.075) for call in fake.calls if call[0] == 'set_pwm_channel_duty_cycle')
    assert fake.calls[-1] == ('set_pwm_enable', False)


def test_depth_from_pressure():
    assert depth_from_pressure(101000.0 + 9806.65, 101000.0, 1000.0) == pytest.approx(1.0)
    assert depth_from_pressure(101000.0 + 9806.65, 101000.0, 1029.0) == pytest.approx(0.9718, abs=1e-3)


def test_zero_of_the_depth_at_start():
    sensor = FakeSensor()
    node = BarometerNode(sensor=sensor)
    node.pub_depth, node.pub_pressure = Capture(), Capture()
    sensor.pressure_pa = 101000.0 + 2.0 * 9806.65
    node.read_sensor()
    assert node.pub_depth.messages[0].data == pytest.approx(2.0)
    assert node.pub_pressure.messages[0].fluid_pressure == pytest.approx(sensor.pressure_pa)
    node.destroy_node()


def test_sensor_not_found():
    with pytest.raises(RuntimeError):
        BarometerNode(sensor=FakeSensor(found=False))
