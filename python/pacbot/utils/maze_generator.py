# python/pacbot/utils/maze_generator.py
import random
import numpy as np
from typing import List, Tuple, Set, Dict
from ..core.maze import Maze
from ..utils.config import CFG

class MazeGenerator:
    @staticmethod
    def generate(width: int = None, height: int = None, seed: int = None) -> Maze:
        if seed is not None: random.seed(seed); np.random.seed(seed)
        w = width or CFG.MAZE_WIDTH
        h = height or CFG.MAZE_HEIGHT
        # Force odd
        w = w if w % 2 == 1 else w + 1
        h = h if h % 2 == 1 else h + 1

        grid = np.ones((h, w), dtype=np.uint8) # 1=Path
        grid[0,:]=0; grid[-1,:]=0; grid[:,0]=0; grid[:,-1]=0

        def divide(x1, y1, x2, y2):
            if x2 - x1 < 3 or y2 - y1 < 3: return
            horizontal = (y2 - y1) > (x2 - x1)
            if horizontal:
                wy = random.randrange(y1 + 1, y2, 2)
                px = random.randrange(x1, x2 + 1, 2)
                grid[wy, x1:x2+1] = 0; grid[wy, px] = 1
                divide(x1, y1, x2, wy)
                divide(x1, wy, x2, y2)
            else:
                wx = random.randrange(x1 + 1, x2, 2)
                py = random.randrange(y1, y2 + 1, 2)
                grid[y1:y2+1, wx] = 0; grid[py, wx] = 1
                divide(x1, y1, wx, y2)
                divide(wx, y1, x2, y2)

        divide(0, 0, w-1, h-1)

        # Collect free cells
        free = [(x,y) for y in range(h) for x in range(w) if grid[y,x]==1]
        if not free: raise RuntimeError("Gen failed")

        # Strategic Placement
        # Start: Top-Left area | Exit: Bottom-Right area (Max Manhattan Dist)
        start_cand = [c for c in free if c[0] < w//3 and c[1] < h//3]
        exit_cand  = [c for c in free if c[0] > 2*w//3 and c[1] > 2*h//3]
        if not start_cand: start_cand = free
        if not exit_cand: exit_cand = free
        
        start = random.choice(start_cand)
        exit_pos = max(exit_cand, key=lambda p: abs(p[0]-start[0]) + abs(p[1]-start[1]))

        # Ghosts: Near center, away from start
        center = (w//2, h//2)
        ghost_cand = sorted(free, key=lambda p: -((p[0]-center[0])**2 + (p[1]-center[1])**2))[:15]
        ghosts = random.sample(ghost_cand, min(4, len(ghost_cand)))

        # Build Layout Strings
        layout = []
        for y in range(h):
            row = ''.join('#' if grid[y,x]==0 else '.' for x in range(w))
            layout.append(list(row)) # Mutable list

        # Overwrite Specials
        layout[start[1]][start[0]] = 'P'
        layout[exit_pos[1]][exit_pos[0]] = 'E'
        for gx, gy in ghosts: layout[gy][gx] = 'G'
        
        # Power Pellets at Dead Ends
        for x, y in free:
            if (x,y) in [start, exit_pos] + ghosts: continue
            deg = sum(1 for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)] if grid[y+dy, x+dx]==1)
            if deg == 1 and random.random() < 0.25: layout[y][x] = 'o'

        return Maze.from_layout([''.join(r) for r in layout])

    @staticmethod
    def generate_suite(count: int = 20) -> List[Maze]:
        """Generates curriculum: Increasing size/difficulty."""
        maps = []
        for i in range(count):
            tier = i // 5
            w = 21 + tier * 6
            h = 21 + tier * 6
            # Ensure different seeds
            m = MazeGenerator.generate(w, h, seed=42 + i)
            maps.append(m)
        return maps
