"""Stop-and-wait decision for moving obstacles (no ROS, unit tested).

- CLEAR -> YIELDING when a moving obstacle is within stop_distance in the
  direction of travel.  While YIELDING every velocity command becomes zero.
- YIELDING -> CLEAR once no moving obstacle is within resume_distance for
  clear_hold seconds.  The larger resume distance and the hold time stop the
  robot creeping stop/go around an obstacle at the threshold.
- YIELDING -> TIMED_OUT after max_wait seconds: the obstacle is left to the
  planner/controller, which route around it (Collision Monitor still guards
  contact).  TIMED_OUT returns to CLEAR when the area clears, re-arming the
  rule for the next encounter.
"""

from dataclasses import dataclass
import math

CLEAR = 'CLEAR'
YIELDING = 'YIELDING'
TIMED_OUT = 'TIMED_OUT'


@dataclass
class YieldConfig:
    stop_distance: float = 1.0
    resume_distance: float = 1.5
    clear_hold: float = 1.0
    max_wait: float = 10.0

    def __post_init__(self):
        if not 0.0 < self.stop_distance < self.resume_distance:
            raise ValueError('need 0 < stop_distance < resume_distance')
        if self.clear_hold < 0.0 or self.max_wait <= 0.0:
            raise ValueError('clear_hold must be >= 0 and max_wait > 0')


class YieldGate:

    def __init__(self, config=None):
        self.config = config or YieldConfig()
        self.state = CLEAR
        self._since = None        # when YIELDING started
        self._clear_since = None  # when the area last became clear

    def update(self, now, obstacles, reversing=False):
        """Return True if motion may continue.

        obstacles: (x, y) of moving obstacles in the robot frame (x forward,
        y left); reversing: the command drives backwards, so "ahead" is
        behind the robot.
        """
        cfg = self.config
        ahead = [o for o in obstacles if (o[0] < 0.0 if reversing else o[0] >= 0.0)]
        blocking = any(math.hypot(x, y) <= cfg.stop_distance for x, y in ahead)
        nearby = any(math.hypot(x, y) <= cfg.resume_distance for x, y in obstacles)

        if nearby:
            self._clear_since = None
        elif self._clear_since is None:
            self._clear_since = now
        cleared = (self._clear_since is not None
                   and now - self._clear_since >= cfg.clear_hold)

        if self.state == CLEAR and blocking:
            self.state, self._since = YIELDING, now
        elif self.state == YIELDING:
            if cleared:
                self.state = CLEAR
            elif now - self._since > cfg.max_wait:
                self.state = TIMED_OUT
        elif self.state == TIMED_OUT and cleared:
            self.state = CLEAR
        return self.state != YIELDING
