# python/pacbot/utils/config.py
import math
from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class SimConfig:
    # Time
    SIM_DT: float = 0.05          # 20 Hz Physics/Logic
    MAX_TIME: float = 120.0       # Seconds
    
    # Maze
    MAZE_WIDTH: int = 31
    MAZE_HEIGHT: int = 31
    CELL_SIZE_PX: int = 20        # Renderer
    
    # Scoring
    PELLET_POINTS: int = 10
    POWER_PELLET_POINTS: int = 50
    EXIT_BONUS: int = 500
    DEATH_PENALTY: int = -10000

    # Physics (Cells/Second)
    PACBOT_MAX_SPEED: float = 3.5
    PACBOT_MAX_ACCEL: float = 10.0
    GHOST_SPEED: float = 2.5
    GHOST_MAX_ACCEL: float = 5.0

    # Decision Engine Weights (TUNE THESE)
    W_PELLET_VALUE: float = 1.0
    W_PATH_COST: float = 1.2
    W_RISK_AVERSION: float = 3.0      # Higher = Scared
    W_TIME_PRESSURE: float = 0.05     # Urgency
    AGGRESSION_DECAY: float = 0.8     # Aggression = time_remaining_ratio * this
    
    # Ghost Tracker
    GHOST_PREDICTION_HORIZON: float = 5.0 # Seconds
    GHOST_PREDICTION_DT: float = 0.1
    DANGER_GAUSSIAN_SIGMA: float = 0.7
    
    # Sensor Noise
    ODOM_DRIFT_STD: float = 0.015     # Per meter
    LIDAR_RANGE_NOISE: float = 0.03
    LIDAR_ANGLE_RES: float = math.radians(3.0)
    GHOST_DETECT_RANGE: float = 8.0
    GHOST_DETECT_FOV: float = math.radians(140.0)
    GHOST_FALSE_POS_RATE: float = 0.005
    GHOST_FALSE_NEG_RATE: float = 0.05

    # Visualization
    FPS: int = 60
    SHOW_DEBUG: bool = True
    SHOW_DANGER_MAP: bool = True

# Global Instance
CFG = SimConfig()
