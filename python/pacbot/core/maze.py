# python/pacbot/core/maze.py
from dataclasses import dataclass, field
from typing import List, Tuple, Set, Dict, Optional
import numpy as np

@dataclass
class Maze:
    width: int
    height: int
    walls: Set[Tuple[int, int]] = field(default_factory=set)
    pellets: Dict[Tuple[int, int], int] = field(default_factory=dict) # pos -> value
    power_pellets: Set[Tuple[int, int]] = field(default_factory=set)
    ghost_spawns: List[Tuple[int, int]] = field(default_factory=list)
    exit_pos: Optional[Tuple[int, int]] = None
    start_pos: Optional[Tuple[int, int]] = None
    
    eaten_pellets: Set[Tuple[int, int]] = field(default_factory=set)
    _grid_cache: Optional[np.ndarray] = field(default=None, repr=False) # 0=free, 1=wall

    @classmethod
    def from_layout(cls, layout: List[str]):
        h = len(layout); w = len(layout[0])
        m = cls(w, h)
        for y, row in enumerate(layout):
            for x, ch in enumerate(row):
                pos = (x, y)
                if ch == '#': m.walls.add(pos)
                elif ch == '.': m.pellets[pos] = 10
                elif ch == 'o': m.pellets[pos] = 50; m.power_pellets.add(pos)
                elif ch == 'G': m.ghost_spawns.append(pos)
                elif ch == 'E': m.exit_pos = pos
                elif ch == 'P': m.start_pos = pos
        return m

    def is_wall(self, x: int, y: int) -> bool:
        return (x, y) in self.walls or not (0 <= x < self.width and 0 <= y < self.height)

    def is_free(self, x: int, y: int) -> bool:
        return not self.is_wall(x, y)

    def get_pellet_value(self, pos: Tuple[int, int]) -> int:
        if pos in self.eaten_pellets: return 0
        return self.pellets.get(pos, 0)

    def eat_pellet(self, pos: Tuple[int, int]) -> int:
        val = self.get_pellet_value(pos)
        if val > 0: self.eaten_pellets.add(pos)
        return val

    def remaining_pellets(self) -> List[Tuple[int, int]]:
        return [p for p in self.pellets if p not in self.eaten_pellets]

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        dirs = [(0,1), (0,-1), (1,0), (-1,0)]
        return [(x+dx, y+dy) for dx, dy in dirs if self.is_free(x+dx, y+dy)]

    def to_numpy_grid(self) -> np.ndarray:
        """Returns HxW uint8 array (0=free, 1=wall) for C++ Planner."""
        if self._grid_cache is None:
            g = np.zeros((self.height, self.width), dtype=np.uint8)
            for (x,y) in self.walls: g[y,x] = 1
            self._grid_cache = g
        return self._grid_cache
