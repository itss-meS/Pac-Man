# python/pacbot/sim/engine.py
import time
from typing import Optional
from ..core.maze import Maze
from ..core.entities import PacBot, Pose, BotState
from ..core.ghost_tracker import GhostTracker
from ..core.decision_engine import DecisionEngine
from ..core.motor_model import MotorController
from ..sensors.sensor_suite import SensorSuite
from ..utils.config import CFG

class SimulationEngine:
    def __init__(self, maze: Maze = None, seed: int = None):
        self.maze = maze or MazeGenerator.generate(seed=seed) # type: ignore
        self.pacbot = PacBot(pose=Pose(*self.maze.start_pos) if self.maze.start_pos else Pose(1.5, 1.5))
        
        self.tracker = GhostTracker(self.maze)
        self.tracker.initialize_ghosts(self.maze.ghost_spawns)
        
        self.decision = DecisionEngine(self.maze, self.tracker)
        self.motor = MotorController(self.maze)
        self.sensors = SensorSuite(self.maze)
        
        self.sim_time = 0.0
        self.running = True
        self.paused = False
        self.game_over = False
        self.win = False
        self.final_score = 0
        self.reason = ""
        self.path_history = []

    def step(self, dt: float = CFG.SIM_DT):
        if self.paused or self.game_over: return
        
        self.sim_time += dt
        time_rem = CFG.MAX_TIME - self.sim_time
        if time_rem <= 0: self._end(False, "Time Out"); return

        # 1. Sense
        believed, ghost_meas = self.sensors.perceive(self.pacbot, self.tracker.ghosts, dt)
        
        # 2. Track Ghosts (Truth + Sensor Fusion)
        self.tracker.update(dt, self.pacbot.pose, ghost_meas)
        
        # 3. Decide
        self.decision.update(self.pacbot, dt, time_rem)
        
        # 4. Act (Motor Control updates True Pose)
        self.motor.update(self.pacbot, dt)
        
        # 5. World Interaction
        self._collect_pellets()
        self._check_collision()
        self._check_exit()
        
        # History
        self.path_history.append((self.pacbot.pose.x, self.pacbot.pose.y))
        if len(self.path_history) > 10000: self.path_history.pop(0)

    def _collect_pellets(self):
        cell = self.pacbot.pose.as_int()
        val = self.maze.eat_pellet(cell)
        if val: self.pacbot.score += val

    def _check_collision(self):
        bp = np.array([self.pacbot.pose.x, self.pacbot.pose.y])
        for g in self.tracker.ghosts.values():
            gp = np.array([g.pose.x, g.pose.y])
            if np.linalg.norm(bp - gp) < 0.6:
                self._end(False, f"Caught by Ghost {g.id}")

    def _check_exit(self):
        if self.maze.exit_pos:
            ex, ey = self.maze.exit_pos
            if abs(self.pacbot.pose.x - ex) < 0.5 and abs(self.pacbot.pose.y - ey) < 0.5:
                self.pacbot.score += CFG.EXIT_BONUS
                self._end(True, "Escaped!")

    def _end(self, win: bool, reason: str):
        self.game_over = True; self.win = win; self.reason = reason
        self.final_score = self.pacbot.score
        print(f"\n=== GAME OVER ===\n{'WIN' if win else 'LOSS'}: {reason}")
        print(f"Time: {self.sim_time:.1f}s | Pellets: {len(self.maze.eaten_pellets)}/{len(self.maze.pellets)}")
        print(f"Score: {self.final_score}\n=================")

    def reset(self, seed=None):
        self.__init__(seed=seed)

# Late import to avoid circular dependency
from ..utils.maze_generator import MazeGenerator
