"""Unit tests of gnc_control.mission_plan and gnc_control.teleop (no ROS needed: python3 -m pytest test/)."""
import pytest

from gnc_control import mission_plan as mp
from gnc_control.teleop import TeleopState


def timed(**kwargs):
    settings = dict(steps=[mp.Step(depth=1.0, heading=90.0, duration=10.0), mp.Step(depth=2.0, heading=180.0, duration=10.0)],
                    start_delay_s=5.0)
    settings.update(kwargs)
    return mp.MissionRunner(mp.Mission(**settings))


def waypoint(**kwargs):
    settings = dict(steps=[mp.Step(depth=2.0, north=10.0, west=0.0, radius=1.0, timeout=20.0)], start_delay_s=0.0)
    settings.update(kwargs)
    return mp.MissionRunner(mp.Mission(**settings))


def at(runner, t, north=0.0, west=0.0, depth=1.0, teleop=False):
    return runner.update(float(t), north, west, depth, teleop)


def running(runner, t=0.0):
    at(runner, t)
    return at(runner, t + runner.mission.start_delay_s)


def test_no_flapping_until_the_estimate_and_the_delay():
    runner = timed()
    assert runner.update(0.0).phase == mp.WAITING
    assert runner.update(100.0).swim is False                           # still no state
    out = at(runner, 101.0)
    assert out.phase == mp.DELAY and out.swim is False
    assert at(runner, 105.9).phase == mp.DELAY
    assert at(runner, 106.0).phase == mp.RUNNING


def test_timed_steps_in_order():
    runner = timed()
    out = running(runner)
    assert (out.heading, out.depth, out.step, out.swim) == (90.0, 1.0, 1, True)
    out = at(runner, 14.9)
    assert out.step == 1
    out = at(runner, 15.0)
    assert (out.heading, out.depth, out.step) == (180.0, 2.0, 2)


def test_after_the_last_step_it_surfaces_then_stops():
    runner = timed()
    running(runner)
    at(runner, 15.0)
    out = at(runner, 25.0, depth=2.0)
    assert out.phase == mp.SURFACING and out.swim and out.depth == 0.5 and out.reason == 'mission completed'
    assert out.heading == 180.0                                         # starts from the last heading
    out = at(runner, 30.0, depth=0.71)
    assert out.phase == mp.SURFACING
    assert out.heading == pytest.approx(280.0)                          # and turns: 20°/s × 5 s
    out = at(runner, 31.0, depth=0.69)
    assert out.phase == mp.FINISHED and out.swim is False


def test_surface_timeout_ends_the_mission():
    runner = timed(surface_timeout_s=10.0)
    running(runner)
    at(runner, 15.0)
    at(runner, 25.0, depth=2.0)
    out = at(runner, 36.0, depth=2.0)
    assert out.phase == mp.FINISHED and 'surface not reached' in out.reason


@pytest.mark.parametrize('north,west,heading', [(10.0, 0.0, 0.0), (0.0, -10.0, 90.0), (-10.0, 0.0, 180.0), (0.0, 10.0, 270.0)])
def test_bearing(north, west, heading):
    assert mp.bearing(north, west) == pytest.approx(heading)


def test_waypoint_aims_at_the_point():
    runner = mp.MissionRunner(mp.Mission(steps=[mp.Step(depth=2.0, north=3.0, west=-4.0, radius=1.0, timeout=20.0)], start_delay_s=0.0))
    out = running(runner)
    assert out.heading == pytest.approx(mp.bearing(3.0, -4.0))          # 3 m north, 4 m east
    assert 0.0 < out.heading < 90.0


def test_waypoint_reached_within_the_radius():
    runner = waypoint()
    running(runner)
    assert at(runner, 5.0, north=8.9).phase == mp.RUNNING
    assert at(runner, 6.0, north=9.1).phase == mp.SURFACING


def test_waypoint_timeout_surfaces():
    runner = waypoint()
    running(runner)
    out = at(runner, 21.0)
    assert out.phase == mp.SURFACING and 'not reached' in out.reason


def test_time_limit_surfaces():
    runner = timed(time_limit_s=3.0, steps=[mp.Step(depth=1.0, heading=0.0, duration=100.0)])
    running(runner)
    out = at(runner, 9.0)
    assert out.phase == mp.SURFACING and 'time limit' in out.reason


def test_depth_limit_surfaces():
    runner = timed(max_depth_m=2.0)
    running(runner)
    out = at(runner, 6.0, depth=2.1)
    assert out.phase == mp.SURFACING and 'depth limit' in out.reason


@pytest.mark.parametrize('t', [6.0, 40.0])
def test_state_lost_stops_at_once(t):
    runner = timed()
    running(runner)
    if t > 30.0:
        at(runner, 15.0)
        assert at(runner, 25.0, depth=2.0).phase == mp.SURFACING
    out = runner.update(t)
    assert out.phase == mp.STOPPED and out.swim is False
    assert at(runner, t + 1.0).phase == mp.STOPPED                      # the state coming back does not restart it


def test_state_lost_when_the_delay_ends_stops():
    runner = timed()
    assert at(runner, 0.0).phase == mp.DELAY
    delay = runner.mission.start_delay_s
    assert runner.update(delay / 2.0).phase == mp.DELAY                 # the tail is stopped: the delay can go on
    out = runner.update(delay + 1.0)
    assert out.phase == mp.STOPPED and out.swim is False and out.reason == 'state lost'


def test_state_back_before_the_delay_ends_runs():
    runner = timed()
    at(runner, 0.0)
    delay = runner.mission.start_delay_s
    runner.update(delay / 2.0)
    assert at(runner, delay + 1.0).phase == mp.RUNNING


@pytest.mark.parametrize('moment', ['waiting', 'running', 'surfacing'])
def test_teleop_takes_over_at_any_moment(moment):
    runner = timed()
    if moment != 'waiting':
        running(runner)
    if moment == 'surfacing':
        at(runner, 15.0)
        at(runner, 25.0, depth=2.0)
    out = at(runner, 26.0, depth=2.0, teleop=True)
    assert out.phase == mp.OVERRIDDEN and out.swim is False
    out = at(runner, 100.0, depth=2.0)                                  # released: does not resume
    assert out.phase == mp.OVERRIDDEN and out.swim is False


def test_parse_mission():
    mission = mp.parse_mission({'start_delay_s': 2, 'steps': [{'heading': 0, 'depth': 1, 'duration': 5}, {'north': 1, 'west': 2, 'depth': 1, 'radius': 1, 'timeout': 5}]})
    assert mission.start_delay_s == 2.0
    assert not mission.steps[0].is_waypoint
    assert mission.steps[1].is_waypoint


@pytest.mark.parametrize('data', [
    {}, {'steps': []}, {'steps': [{'heading': 0, 'depth': 1}]},
    {'steps': [{'north': 1, 'depth': 1}]}, {'steps': [{'heading': 0, 'depth': 9, 'duration': 1}]},
    {'steps': [{'heading': 0, 'depth': 0.1, 'duration': 1}]}, {'steps': [{'heading': 0, 'depth': 1, 'duration': 1, 'speed': 1}]},
    {'start': 1, 'steps': [{'heading': 0, 'depth': 1, 'duration': 1}]},
])
def test_wrong_missions_are_refused(data):
    with pytest.raises(ValueError):
        mp.parse_mission(data)


def teleop():
    return TeleopState(1.5, 15.0, 3.0, 60.0)


def test_teleop_starts_released():
    state = teleop()
    assert not state.active
    assert state.key('z') is False
    assert not state.active


@pytest.mark.parametrize('key', [' ', 'a', 'd', 'w', 's', 'c', 'x', 'A'])
def test_any_control_key_takes_the_robot(key):
    state = teleop()
    assert state.key(key)
    assert state.active


def test_teleop_commands():
    state = teleop()
    state.key(' ')
    state.key('d')
    state.key('w')
    assert state.commands() == pytest.approx((0.5, 0.25, 5.0 / 60.0, 0.1))
    state.key('x')
    assert state.commands() == pytest.approx((0.0, 0.0, 0.0, 0.0))
    assert state.active


def test_teleop_limits():
    state = teleop()
    for _ in range(50):
        state.key('d')
        state.key('w')
    assert state.bias_deg == 30.0
    assert state.mass == 1.0


def test_release():
    state = teleop()
    state.key(' ')
    state.key('r')
    assert not state.active and not state.swimming
