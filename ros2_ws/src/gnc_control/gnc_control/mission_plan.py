"""Mission of the robot fish: a list of steps and the logic to run it (pure Python, no ROS, so it can be unit tested).

The robot cannot be reached under water, so a mission must run alone and finish safely. Phases:

    WAITING → DELAY → RUNNING → SURFACING → FINISHED

  WAITING    no estimate of the state yet; the tail does not flap.
  DELAY      `start_delay_s`, the time to put the robot in the water; the tail does not flap.
  RUNNING    the steps, one after the other. Two kinds of steps:
               timed:    {heading, depth, duration}          keeps a heading for some time; does not use the position
               waypoint: {north, west, depth, radius, timeout}  aims at a point until it is within `radius`
  SURFACING  swims up to `surface_depth_m`, because a neutral robot that stops swimming does not rise by itself. It
             turns in circles while it rises: going straight could end against a wall of the basin, where it would stay.
  FINISHED   the tail stops.

Two phases end at once, with the tail stopped:
  STOPPED     the state was lost: without knowing where it is, the robot must not keep swimming.
  OVERRIDDEN  an operator took the robot with the teleop. The mission does not resume by itself afterwards.

These events interrupt the steps and go to SURFACING: a waypoint that is not reached in its `timeout`, the time limit
of the whole mission, and a depth beyond `max_depth_m`.
"""
import math
from dataclasses import dataclass
from typing import List, Optional

WAITING, DELAY, RUNNING, SURFACING, FINISHED, STOPPED, OVERRIDDEN = (
    'WAITING', 'DELAY', 'RUNNING', 'SURFACING', 'FINISHED', 'STOPPED', 'OVERRIDDEN')
ENDED = (FINISHED, STOPPED, OVERRIDDEN)


@dataclass
class Step:
    depth: float                                        # [m]
    heading: Optional[float] = None                     # timed step [deg, clockwise from true north]
    duration: Optional[float] = None                    # timed step [s]
    north: Optional[float] = None                       # waypoint [m], origin at the start
    west: Optional[float] = None                        # waypoint [m]
    radius: Optional[float] = None                      # waypoint reached within this distance [m]
    timeout: Optional[float] = None                     # waypoint: longest time allowed [s]

    @property
    def is_waypoint(self):
        return self.north is not None


@dataclass
class Mission:
    steps: List[Step]
    start_delay_s: float = 30.0
    time_limit_s: float = 300.0                         # all the steps together
    max_depth_m: float = 4.5
    surface_depth_m: float = 0.5
    surface_margin_m: float = 0.2                       # surfacing ends at surface_depth_m + this margin
    surface_timeout_s: float = 60.0
    surface_turn_rate_deg_s: float = 20.0               # about 2 m of radius at 0.7 m/s


@dataclass
class Output:
    phase: str
    swim: bool                                          # False: the tail must not flap
    heading: Optional[float] = None                     # None: nothing to command
    depth: Optional[float] = None
    step: int = 0                                       # 1 = first step, 0 = none
    reason: str = ''


def parse_mission(data):
    """Builds a Mission from a dictionary (a YAML file). Raises ValueError with a clear message if it is wrong."""
    if not isinstance(data, dict) or not data.get('steps'):
        raise ValueError('a mission needs a list of steps')
    settings = {k: float(v) for k, v in data.items() if k != 'steps'}
    allowed = ('start_delay_s', 'time_limit_s', 'max_depth_m', 'surface_depth_m', 'surface_margin_m', 'surface_timeout_s',
               'surface_turn_rate_deg_s')
    for key in settings:
        if key not in allowed:
            raise ValueError(f'unknown mission setting: {key}')
    mission = Mission(steps=[], **settings)
    for number, raw in enumerate(data['steps'], 1):
        unknown = set(raw) - {'depth', 'heading', 'duration', 'north', 'west', 'radius', 'timeout'}
        if unknown:
            raise ValueError(f'step {number}: unknown keys {sorted(unknown)}')
        step = Step(**{k: float(v) for k, v in raw.items()})
        needed = ('north', 'west', 'radius', 'timeout') if 'north' in raw or 'west' in raw else ('heading', 'duration')
        missing = [k for k in needed + ('depth',) if k not in raw]
        if missing:
            raise ValueError(f'step {number}: missing {missing}')
        if not mission.surface_depth_m <= step.depth <= mission.max_depth_m:
            raise ValueError(f'step {number}: depth {step.depth} outside '
                             f'[{mission.surface_depth_m}, {mission.max_depth_m}]')
        mission.steps.append(step)
    return mission


def bearing(north, west):
    """Heading [deg, clockwise from north, 0–360) of a displacement given in north and west."""
    return math.degrees(math.atan2(-west, north)) % 360.0


class MissionRunner:
    def __init__(self, mission):
        self.mission = mission
        self.phase = WAITING
        self.reason = ''
        self.index = 0                                  # current step
        self.delay_start = None                         # [s]
        self.mission_start = None                       # [s]
        self.step_start = None                          # [s]
        self.surfacing_start = None                     # [s]
        self.heading = 0.0                              # last heading asked [deg]

    def _surface(self, now, reason):
        self.phase, self.reason, self.surfacing_start = SURFACING, reason, now

    def _next_step(self, now):
        self.index += 1
        self.step_start = now
        if self.index >= len(self.mission.steps):
            self._surface(now, 'mission completed')

    def update(self, now, north=None, west=None, depth=None, teleop=False):
        """One step at the time `now` [s]. north, west, depth: the current state, None if it is lost."""
        has_state = north is not None and west is not None and depth is not None
        if self.phase == WAITING and has_state:
            self.phase, self.delay_start = DELAY, now
        if self.phase == DELAY and now - self.delay_start >= self.mission.start_delay_s:
            self.phase, self.mission_start, self.step_start = RUNNING, now, now

        # After the changes of phase above: the state can be lost at the moment the delay ends.
        if self.phase not in ENDED:
            if teleop:
                self.phase, self.reason = OVERRIDDEN, 'taken by the teleop'
            elif self.phase in (RUNNING, SURFACING) and not has_state:
                self.phase, self.reason = STOPPED, 'state lost'

        if self.phase == RUNNING:
            step = self.mission.steps[self.index]
            elapsed = now - self.step_start
            if depth > self.mission.max_depth_m:
                self._surface(now, f'depth limit: {depth:.2f} m')
            elif now - self.mission_start > self.mission.time_limit_s:
                self._surface(now, 'time limit of the mission')
            elif step.is_waypoint:
                if math.hypot(step.north - north, step.west - west) <= step.radius:
                    self._next_step(now)
                elif elapsed > step.timeout:
                    self._surface(now, f'waypoint {self.index + 1} not reached in {step.timeout:.0f} s')
            elif elapsed >= step.duration:
                self._next_step(now)

        if self.phase == SURFACING:
            target = self.mission.surface_depth_m + self.mission.surface_margin_m
            if depth <= target:
                self.phase = FINISHED
            elif now - self.surfacing_start > self.mission.surface_timeout_s:
                self.phase, self.reason = FINISHED, self.reason + '; surface not reached in time'

        return self._output(now, north, west)

    def _output(self, now, north, west):
        if self.phase == RUNNING:
            step = self.mission.steps[self.index]
            if step.is_waypoint:
                self.heading = bearing(step.north - north, step.west - west)
            else:
                self.heading = step.heading
            return Output(RUNNING, True, self.heading, step.depth, self.index + 1)
        if self.phase == SURFACING:
            turned = self.mission.surface_turn_rate_deg_s * (now - self.surfacing_start)
            return Output(SURFACING, True, (self.heading + turned) % 360.0, self.mission.surface_depth_m, reason=self.reason)
        return Output(self.phase, False, reason=self.reason)
