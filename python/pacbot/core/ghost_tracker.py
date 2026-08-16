# python/pacbot/core/ghost_tracker.py
import numpy as np
from typing import Dict, List, Tuple, Optional
from ..core.maze import Maze
from ..core.entities import Ghost, Pose, GhostState
from ..utils.config import CFG

try:
    import pacbot_core as core_cpp
    CPP_KF = True
except ImportError:
    CPP_KF = False

class GhostTracker:
    def __init__(self, maze: Maze):
        self.maze = maze
        self.ghosts: Dict[int, Ghost] = {}
        self.danger_map: Optional[np.ndarray] = None # H x W x T
        self.time_steps = int(CFG.GHOST_PREDICTION_HORIZON / CFG.GHOST_PREDICTION_DT)
        self.ekfs: Dict[int, 'core_cpp.GhostEKF'] = {} if CPP_KF else None

    def initialize_ghosts(self, spawn_positions: List[Tuple[int, int]]):
        for i, pos in enumerate(spawn_positions):
            g = Ghost(id=i, pose=Pose(float(pos[0])+0.5, float(pos[1])+0.5))
            g.patrol_points = self._gen_patrol(pos)
            self.ghosts[i] = g
            if CPP_KF:
                self.ekfs[i] = core_cpp.GhostEKF(dt=CFG.SIM_DT)
                # Initialize EKF with spawn pos
                self.ekfs[i].initialize(np.array([g.pose.x, g.pose.y], dtype=np.float64))

    def _gen_patrol(self, start: Tuple[int,int]) -> List[Tuple[int,int]]:
        # Simple 4-point loop around spawn
        x, y = start
        pts = []
        for dx, dy in [(2,0), (0,2), (-2,0), (0,-2)]:
            nx, ny = x+dx, y+dy
            if self.maze.is_free(nx, ny): pts.append((nx, ny))
        return pts if pts else [start]

    def update(self, dt: float, pacbot_pose: Pose, sensor_measurements: Dict[int, Pose]):
        # 1. Physics Update (Truth) & Intent Inference
        for g in self.ghosts.values():
            self._physics_step(g, dt)
            self._infer_intent(g, pacbot_pose)
            self._execute_behavior(g, dt, pacbot_pose)

        # 2. Sensor Fusion (EKF Update)
        if CPP_KF:
            for gid, meas in sensor_measurements.items():
                if gid in self.ekfs:
                    z = np.array([meas.x, meas.y], dtype=np.float64)
                    # Measurement Covariance (Range dependent)
                    R = np.eye(2) * (0.05 + 0.01 * pacbot_pose.dist_to_pt((meas.x, meas.y)))
                    self.ekfs[gid].update(z, R)
                    # Sync Truth -> EKF State (for prediction)
                    x_est = self.ekfs[gid].get_state()
                    self.ghosts[gid].pose.x, self.ghosts[gid].pose.y = x_est[0], x_est[1]

        # 3. Predict Trajectories (For Danger Map)
        for g in self.ghosts.values():
            if CPP_KF and g.id in self.ekfs:
                g.predicted_traj = [(p.x, p.y) for p in self.ekfs[g.id].predict_trajectory(CFG.GHOST_PREDICTION_HORIZON, CFG.GHOST_PREDICTION_DT)]
            else:
                g.predicted_traj = self._simple_predict(g)

        # 4. Build Danger Map
        self._build_danger_map()

    def _physics_step(self, g: Ghost, dt: float):
        g.pose.x += g.velocity * np.cos(g.pose.theta) * dt
        g.pose.y += g.velocity * np.sin(g.pose.theta) * dt

    def _infer_intent(self, g: Ghost, pb: Pose):
        dist = g.pose.dist_to(pb)
        los = self._los(g.pose, pb)
        if dist < 3.5: g.state = GhostState.CHASE
        elif dist < 7.0 and los: g.state = GhostState.AMBUSH
        elif g.patrol_points: g.state = GhostState.PATROL
        else: g.state = GhostState.RANDOM

    def _los(self, p1: Pose, p2: Pose) -> bool:
        x0, y0 = int(p1.x), int(p1.y); x1, y1 = int(p2.x), int(p2.y)
        dx, dy = abs(x1-x0), abs(y1-y0)
        sx, sy = 1 if x0<x1 else -1, 1 if y0<y1 else -1
        err = dx - dy
        while True:
            if self.maze.is_wall(x0, y0): return False
            if x0==x1 and y0==y1: break
            e2 = 2*err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 < dx: err += dx; y0 += sy
        return True

    def _execute_behavior(self, g: Ghost, dt: float, pb: Pose):
        target = None; speed = CFG.GHOST_SPEED
        if g.state == GhostState.CHASE:
            target = (pb.x, pb.y); speed *= 1.2
        elif g.state == GhostState.AMBUSH:
            target = (pb.x, pb.y) # Simple: aim at current
        elif g.state == GhostState.PATROL:
            target = g.patrol_points[g.patrol_idx]
            if g.pose.dist_to_pt(target) < 0.5: g.patrol_idx = (g.patrol_idx + 1) % len(g.patrol_points)
        else:
            nbrs = self.maze.get_neighbors(g.pose.as_int())
            if nbrs: target = np.random.choice(nbrs)
        
        if target:
            dx, dy = target[0] - g.pose.x, target[1] - g.pose.y
            tgt_angle = np.arctan2(dy, dx)
            diff = (tgt_angle - g.pose.theta + np.pi) % (2*np.pi) - np.pi
            turn = np.clip(diff, -3.0*dt, 3.0*dt)
            g.pose.theta += turn
            g.velocity = speed
        else: g.velocity = 0

    def _simple_predict(self, g: Ghost) -> List[Tuple[float,float]]:
        traj = []; x,y,th = g.pose.x, g.pose.y, g.pose.theta; v = g.velocity
        for _ in range(self.time_steps):
            traj.append((x,y))
            x += v * np.cos(th) * CFG.GHOST_PREDICTION_DT
            y += v * np.sin(th) * CFG.GHOST_PREDICTION_DT
        return traj

    def _build_danger_map(self):
        H, W = self.maze.height, self.maze.width
        T = self.time_steps
        danger = np.zeros((H, W, T), dtype=np.float32)
        
        for g in self.ghosts.values():
            for t_idx, (px, py) in enumerate(g.predicted_traj):
                ix, iy = int(round(px)), int(round(py))
                if 0 <= ix < W and 0 <= iy < H:
                    # Gaussian Splat
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            nx, ny = ix+dx, iy+dy
                            if 0 <= nx < W and 0 <= ny < H:
                                d2 = dx*dx + dy*dy
                                prob = np.exp(-d2 / (2 * CFG.DANGER_GAUSSIAN_SIGMA**2)) * (1.0 - t_idx/T * 0.5)
                                danger[ny, nx, t_idx] = max(danger[ny, nx, t_idx], prob)
        self.danger_map = danger

    def get_danger_map(self, time_idx: int = 0) -> np.ndarray:
        if self.danger_map is None: return np.zeros((self.maze.height, self.maze.width), dtype=np.float32)
        if self.danger_map.ndim == 3:
            idx = min(time_idx, self.danger_map.shape[2]-1)
            return self.danger_map[:, :, idx]
        return self.danger_map

    def get_ghost_poses(self) -> Dict[int, Pose]:
        return {g.id: g.pose for g in self.ghosts.values()}
