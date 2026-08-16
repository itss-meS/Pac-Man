# python/pacbot/sensors/sensor_suite.py
import numpy as np
from typing import Dict, Tuple
from ..core.maze import Maze
from ..core.entities import PacBot, Ghost, Pose
from ..utils.config import CFG

class SensorSuite:
    def __init__(self, maze: Maze):
        self.maze = maze
        self.odo_drift = np.zeros(3) # x, y, theta

    def perceive(self, bot: PacBot, ghosts: Dict[int, Ghost], dt: float) -> Tuple[Pose, Dict[int, Pose]]:
        # 1. Odometry (Dead Reckoning + Noise)
        believed = self._update_odometry(bot, dt)
        
        # 2. Exteroceptive Correction (Simulated Lidar Slam / Wall Snapping)
        believed = self._correct_with_walls(believed)
        
        # 3. Ghost Detection (FOV, Range, Noise)
        ghost_meas = self._detect_ghosts(believed, ghosts)
        
        bot.believed_pose = believed
        bot.believed_ghosts = ghost_meas
        return believed, ghost_meas

    def _update_odometry(self, bot: PacBot, dt: float) -> Pose:
        # True Motion
        dx = bot.velocity * np.cos(bot.pose.theta) * dt
        dy = bot.velocity * np.sin(bot.pose.theta) * dt
        dth = bot.omega * dt
        
        # Drift Accumulation (Random Walk)
        n_xy = CFG.ODOM_DRIFT_STD * np.sqrt(bot.velocity * dt + 1e-6)
        n_th = CFG.ODOM_DRIFT_STD * 0.5 * np.sqrt(dt)
        self.odo_drift[0] += np.random.normal(0, n_xy)
        self.odo_drift[1] += np.random.normal(0, n_xy)
        self.odo_drift[2] += np.random.normal(0, n_th)
        
        return Pose(
            bot.pose.x + self.odo_drift[0],
            bot.pose.y + self.odo_drift[1],
            bot.pose.theta + self.odo_drift[2]
        )

    def _correct_with_walls(self, believed: Pose) -> Pose:
        # Simplified: Snap to corridor center if very close to wall
        # Real impl: Scan Matching / EKF with Lidar
        ix, iy = int(round(believed.x)), int(round(believed.y))
        if self.maze.is_free(ix, iy):
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                if self.maze.is_wall(ix+dx, iy+dy):
                    # Push away from wall center
                    wall_cx, wall_cy = ix+dx+0.5, iy+dy+0.5
                    vx, vy = believed.x - wall_cx, believed.y - wall_cy
                    dist = np.hypot(vx, vy)
                    if dist < 0.45: # Robot Radius ~0.35 + margin
                        push = (0.45 - dist) / dist
                        believed.x += vx * push * 0.5
                        believed.y += vy * push * 0.5
        return believed

    def _detect_ghosts(self, bot_pose: Pose, ghosts: Dict[int, Ghost]) -> Dict[int, Pose]:
        meas = {}
        for gid, g in ghosts.items():
            dx, dy = g.pose.x - bot_pose.x, g.pose.y - bot_pose.y
            dist = np.hypot(dx, dy)
            ang = np.arctan2(dy, dx)
            rel_ang = (ang - bot_pose.theta + np.pi) % (2*np.pi) - np.pi
            
            if abs(rel_ang) > CFG.GHOST_DETECT_FOV / 2: continue
            if dist > CFG.GHOST_DETECT_RANGE: continue
            if np.random.random() < CFG.GHOST_FALSE_NEG_RATE: continue
            
            # Noisy Measurement
            m_dist = dist + np.random.normal(0, CFG.LIDAR_RANGE_NOISE)
            m_ang = rel_ang + np.random.normal(0, CFG.LIDAR_ANGLE_RES)
            
            wx = bot_pose.x + m_dist * np.cos(bot_pose.theta + m_ang)
            wy = bot_pose.y + m_dist * np.sin(bot_pose.theta + m_ang)
            meas[gid] = Pose(wx, wy)
        
        # False Positive
        if np.random.random() < CFG.GHOST_FALSE_POS_RATE:
            meas[-1] = Pose(bot_pose.x + np.random.uniform(-3,3), bot_pose.y + np.random.uniform(-3,3))
        return meas
