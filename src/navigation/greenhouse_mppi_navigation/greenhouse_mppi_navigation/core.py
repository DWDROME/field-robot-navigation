"""ROS-independent mission and command lifecycle contracts."""
from dataclasses import dataclass, field
import math


def finite_pose(values):
    return len(values) == 7 and all(math.isfinite(v) for v in values) and abs(
        sum(v*v for v in values[3:])-1) < 1e-3


@dataclass
class Mission:
    generation: int = 0
    goals: list = field(default_factory=list)
    index: int = 0
    state: str = "idle"
    reason: str = ""
    started: float = 0
    token: int = 0

    def start(self, goals, now):
        if self.state in {"planning", "controlling"}:
            raise ValueError("another mission is active")
        if not goals or len(goals) > 1000 or not all(finite_pose(g) for g in goals):
            raise ValueError("mission requires 1..1000 finite normalized poses")
        self.generation += 1
        self.goals = list(goals)
        self.index = 0
        self.state, self.reason, self.started = "planning", "", now
        return self.generation

    def terminate(self, generation, state, reason=""):
        if generation != self.generation or self.state not in {"planning", "controlling"}:
            return False
        if state not in {"failed", "timeout", "cancelled"}:
            raise ValueError("invalid terminal state")
        self.state, self.reason = state, reason
        return True

    def arrived(self, generation, position, now, xy_tolerance, yaw=None, yaw_tolerance=math.pi):
        if generation != self.generation or self.state not in {"planning", "controlling"}:
            return False
        goal = self.goals[self.index]
        if not all(math.isfinite(v) for v in position) or math.hypot(position[0]-goal[0], position[1]-goal[1]) > xy_tolerance:
            return False
        if yaw is not None:
            x,y,z,w=goal[3:]
            desired=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            if not math.isfinite(yaw) or abs(math.atan2(math.sin(yaw-desired),math.cos(yaw-desired))) > yaw_tolerance:
                return False
        self.index += 1
        self.started = now
        self.state = "completed" if self.index == len(self.goals) else "planning"
        return True


@dataclass
class Freshness:
    """Both transport receipt and source time must advance; enable changes fence old commands."""
    timeout: float
    receipt: float = -math.inf
    stamp: float = -math.inf

    def update(self, receipt, stamp, ros_now):
        if not all(math.isfinite(v) for v in (receipt,stamp,ros_now)) or stamp<=0 or not -0.1 <= ros_now-stamp <= self.timeout:
            self.receipt = -math.inf
            return False
        if stamp < self.stamp:
            self.receipt = -math.inf
            return False
        if stamp == self.stamp:
            return False
        self.receipt, self.stamp = receipt, stamp
        return True

    def valid(self, now, ros_now):
        return 0 <= now-self.receipt <= self.timeout and -0.1 <= ros_now-self.stamp <= self.timeout


class CommandGuard:
    def __init__(self, command_timeout=.2, odom_timeout=.5, terrain_timeout=.7, enable_timeout=.3):
        self.command=Freshness(command_timeout)
        self.odom=Freshness(odom_timeout)
        self.terrain=Freshness(terrain_timeout)
        self.enable_timeout=enable_timeout
        self.enable_receipt=-math.inf
        self.enabled=False
        self.fence=math.inf
        self.velocity=(0.,0.)

    def enable(self, active, now, ros_now):
        if active and not self.enabled:
            self.fence=ros_now
            self.command.receipt=-math.inf
        if not active:
            self.fence=math.inf
            self.command.receipt=-math.inf
        self.enabled=active
        self.enable_receipt=now

    def accept(self, velocity, stamp, now, ros_now):
        if not all(math.isfinite(v) for v in velocity):
            self.command.receipt=-math.inf
            return False
        if stamp < self.fence:
            return False
        if self.command.update(now,stamp,ros_now):
            self.velocity=velocity
            return True
        return False

    def output(self, now, ros_now, tf_ready=True, owners=1, estop=False):
        allowed=self.enabled and 0 <= now-self.enable_receipt <= self.enable_timeout and tf_ready and owners==1 and not estop
        allowed=allowed and all(v.valid(now,ros_now) for v in (self.command,self.odom,self.terrain))
        return self.velocity if allowed else (0.,0.)
