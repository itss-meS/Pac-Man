# python/pacbot/core/pathfinding.py
import numpy as np
from typing import List, Tuple, Optional, Callable
from ..core.maze import Maze
from ..utils.config import CFG

try:
    import pacbot_core as core_cpp
    CPP_AVAILABLE = True
except ImportError:
    CPP_AVAILABLE = False
    print("[WARN] pacbot_core C++ module not found. Falling back to Python A* (Slow).")

def _fallback_astar(maze: Maze, start: Tuple[int,int], goal: Tuple[int,int]) -> Optional[List[Tuple[int,int]]]:
    """Pure Python A* for fallback."""
    import heapq
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    while open_set:
        _, g, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [cur]
            while cur in came_from: cur = came_from[cur]; path.append(cur)
            return path[::-1]
        for nxt in maze.get_neighbors(cur):
            ng = g + 1
            if nxt not in g_score or ng < g_score[nxt]:
                g_score[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_set, (ng + h(nxt, goal), ng, nxt))
    return None

def astar(maze: Maze, 
          start: Tuple[int, int], 
          goal: Tuple[int, int], 
          danger_map: np.ndarray = None, # HxW float32
          danger_weight: float = CFG.W_RISK_AVERSION * 10) -> Optional[List[Tuple[int, int]]]:
    
    if start == goal: return [start]
    
    grid = maze.to_numpy_grid() # Cached
    
    if CPP_AVAILABLE:
        # Prepare Danger Map for C++ (Must be contiguous C-order float64)
        dm_cpp = None
        if danger_map is not None:
            dm_cpp = np.ascontiguousarray(danger_map, dtype=np.float64)
        
        res = core_cpp.astar_plan(
            grid, 
            np.array(start, dtype=np.int32), 
            np.array(goal, dtype=np.int32),
            dm_cpp, 
            float(danger_weight)
        )
        if res['success']:
            return [tuple(p) for p in res['path']]
        return None
    else:
        return _fallback_astar(maze, start, goal)

def find_path_to_exit(maze: Maze, start: Tuple[int,int], danger_map: np.ndarray) -> Optional[List[Tuple[int,int]]]:
    if not maze.exit_pos: return None
    return astar(maze, start, maze.exit_pos, danger_map, danger_weight=100.0) # High weight = Safety First

def find_path_to_target(maze: Maze, start: Tuple[int,int], target: Tuple[int,int], danger_map: np.ndarray) -> Optional[List[Tuple[int,int]]]:
    return astar(maze, start, target, danger_map, danger_weight=50.0) # Medium weight

def find_safest_spot(maze: Maze, start: Tuple[int,int], danger_map: np.ndarray) -> Optional[List[Tuple[int,int]]]:
    """BFS for nearest low-danger cell."""
    from collections import deque
    q = deque([(start, 0)]); visited = {start}
    best = start; best_d = float('inf')
    while q:
        pos, dist = q.popleft()
        d = danger_map[pos[1], pos[0]] if danger_map is not None else 0
        if d < best_d: best_d, best = d, pos
        if dist > 15: continue
        for n in maze.get_neighbors(pos):
            if n not in visited: visited.add(n); q.append((n, dist+1))
    if best == start: return [start]
    return astar(maze, start, best, danger_map, danger_weight=10.0)
