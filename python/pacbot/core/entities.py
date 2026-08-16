# python/pacbot/core/entities.py
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from enum import Enum, auto
import numpy as np

class BotState(Enum):
    EXPLORE = auto()
    TARGET_PELLET = auto()
    EVADE = auto()
    RUSH_EXIT = auto()
    TRAPPED = auto()
    FINISHED = auto()

class GhostState(Enum):
    PATROL = auto()
    CHASE = auto()
    AMBUSH = auto()
    RANDOM = auto()

@dataclass
class Pose:
    x: float; y: float; theta: float = 0.0
    def as_tuple(self) -> Tuple[float, float]: return (self.x, self.y)
    def as_int(self) -> Tuple[int, int]: return (int(round(self.x)), int(round(self.y)))
    def dist_to(self, o: 'Pose') -> float: return np.hypot(self.x-o.x, self.y-o.y)
    def dist_to_pt(self, pt: Tuple[float,float]) -> float: return np.hypot(self.x-pt[0], self.y-pt[1])
    def copy(self): return Pose(self.x, self.y, self.theta)

@dataclass
class PacBot:
    pose: Pose
    believed_pose: Pose = field(default_factory=lambda: Pose(0,0))
    velocity: float = 0.0
    omega: float = 0.0
    score: int = 0
    state: BotState = BotState.EXPLORE
    target: Optional[Tuple[int, int]] = None
    path: List[Tuple[int, int]] = field(default_factory=list)
    believed_ghosts: Dict[int, Pose] = field(default_factory=dict)
    # Motor Interface
    wheel_vel: Tuple[float, float] = (0.0, 0.0)

@dataclass
class Ghost:
    id: int
    pose: Pose
    velocity: float = 0.0
    state: GhostState = GhostState.PATROL
    patrol_points: List[Tuple[int, int]] = field(default_factory=list)
    patrol_idx: int = 0
    target: Optional[Tuple[int, int]] = None
    # C++ EKF Predictions
    predicted_traj: List[Tuple[float, float]] = field(default_factory=list)
