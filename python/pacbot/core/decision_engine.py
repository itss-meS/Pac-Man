# python/pacbot/core/decision_engine.py
import numpy as np
from typing import List, Tuple, Optional, Dict
from ..core.maze import Maze
from ..core.entities import PacBot, BotState, Pose
from ..core.pathfinding import find_path_to_exit, find_path_to_target, find_safest_spot
from ..core.ghost_tracker import GhostTracker
from ..utils.config import CFG

class DecisionEngine:
    def __init__(self, maze: Maze, tracker: GhostTracker):
        self.maze = maze
        self.tracker = tracker
        self.state = BotState.EXPLORE
        self.aggression = 1.0
        self.last_target: Optional[Tuple[int,int]] = None
        self.stuck_timer = 0
        self.pellet_clusters: List[Dict] = []

    def update(self, bot: PacBot, dt: float, time_rem: float):
        # 1. Aggression Decay
        self.aggression = max(0.0, (time_rem / CFG.MAX_TIME) * CFG.AGGRESSION_DECAY)
        
        # 2. Local Danger
        local_danger = self._get_danger_at(bot.believed_pose)
        
        # 3. FSM Transition
        new_state = self._decide_state(bot, local_danger, time_rem)
        if new_state != self.state:
            self.stuck_timer = 0
            # print(f"[FSM] {self.state.name} -> {new_state.name} | Aggro: {self.aggression:.2f}")
        self.state = new_state
        bot.state = self.state

        # 4. Execute Behavior
        self._execute(bot, time_rem)

        # 5. Stuck Detection
        if bot.path and bot.pose.dist_to_pt((bot.path[0][0]+0.5, bot.path[0][1]+0.5)) < 0.3:
            self.stuck_timer = 0
        else:
            self.stuck_timer += 1
        if self.stuck_timer > 40: # 2 sec
            self.state = BotState.TRAPPED

    def _get_danger_at(self, pose: Pose) -> float:
        dmap = self.tracker.get_danger_map(0)
        ix, iy = int(pose.x), int(pose.y)
        if 0 <= iy < dmap.shape[0] and 0 <= ix < dmap.shape[1]:
            return dmap[iy, ix]
        return 1.0

    def _decide_state(self, bot: PacBot, danger: float, time_rem: float) -> BotState:
        # Survival
        if danger > 0.65: return BotState.EVADE
        if self.state == BotState.TRAPPED: return BotState.TRAPPED
        
        # Exit Conditions
        pellets_left = len(self.maze.remaining_pellets())
        dist_exit = abs(bot.believed_pose.x - self.maze.exit_pos[0]) + abs(bot.believed_pose.y - self.maze.exit_pos[1])
        time_to_exit = dist_exit / CFG.PACBOT_MAX_SPEED * 1.5 # Buffer
        
        if time_rem < time_to_exit: return BotState.RUSH_EXIT
        if pellets_left == 0: return BotState.RUSH_EXIT
        
        # Collection
        if self.aggression > 0.25 and pellets_left > 0:
            return BotState.TARGET_PELLET
        
        return BotState.EXPLORE

    def _execute(self, bot: PacBot, time_rem: float):
        dmap = self.tracker.get_danger_map(0)
        start = bot.believed_pose.as_int()

        if self.state == BotState.RUSH_EXIT:
            path = find_path_to_exit(self.maze, start, dmap)
            if path: bot.path, bot.target = path, self.maze.exit_pos
            else: self.state = BotState.TRAPPED

        elif self.state == BotState.TARGET_PELLET:
            target = self._select_target(bot, dmap, time_rem)
|
