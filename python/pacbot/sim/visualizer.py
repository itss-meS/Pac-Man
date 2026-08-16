# python/pacbot/sim/visualizer.py
import pygame
import numpy as np
from ..sim.engine import SimulationEngine
from ..core.entities import Pose, BotState, GhostState
from ..utils.config import CFG, COLORS # COLORS defined below

# Color Constants
COLORS = {
    'bg': (15, 15, 35), 'wall': (50, 50, 90), 'pellet': (255, 215, 0),
    'power': (255, 100, 200), 'exit': (0, 255, 120), 'bot': (0, 220, 255),
    'trail': (0, 150, 200, 80), 'ghost': (255, 60, 60), 'ghost_pred': (255, 100, 100, 40),
    'path': (100, 255, 100), 'target': (255, 255, 0), 'text': (230, 230, 230),
    'ui_bg': (0, 0, 0, 200), 'danger': (255, 0, 0)
}

class Visualizer:
    def __init__(self, engine: SimulationEngine):
        self.engine = engine
        self.cs = CFG.CELL_SIZE_PX
        self.w = engine.maze.width * self.cs
        self.h = engine.maze.height * self.cs + 120
        pygame.init()
        self.screen = pygame.display.set_mode((self.w, self.h), pygame.RESIZABLE)
        pygame.display.set_caption("PacBot Competition Sim | [SPACE] Pause [S] Step [R] Reset [V] Debug [1-4] Strat")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Consolas', 13)
        self.bfont = pygame.font.SysFont('Consolas', 22, bold=True)
        self.show_debug = CFG.SHOW_DEBUG
        self.show_danger = CFG.SHOW_DANGER_MAP

    def handle_events(self) -> bool:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return False
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: return False
                if e.key == pygame.K_SPACE: self.engine.paused = not self.engine.paused
                if e.key == pygame.K_s and self.engine.paused: self.engine.step()
                if e.key == pygame.K_r: self.engine.reset()
                if e.key == pygame.K_v: self.show_debug = not self.show_debug
                if e.key == pygame.K_d: self.show_danger = not self.show_danger
                # Strategy Hotkeys
                if e.key == pygame.K_1: self.engine.decision.aggression = 1.0; print("Strat: GREEDY")
                if e.key == pygame.K_2: self.engine.decision.aggression = 0.6; print("Strat: BALANCED")
                if e.key == pygame.K_3: self.engine.decision.aggression = 0.2; print("Strat: SURVIVAL")
                if e.key == pygame.K_4: self.engine.decision.aggression = 0.0; print("Strat: EXIT ONLY")
            if e.type == pygame.VIDEORESIZE:
                self.w, self.h = e.w, e.h
                self.screen = pygame.display.set_mode((self.w, self.h), pygame.RESIZABLE)
        return True

    def render(self):
        self.screen.fill(COLORS['bg'])
        cs = self.cs
        
        # Danger Map
        if self.show_danger:
            self._render_danger()
        
        # Maze
        self._render_maze()
        
        # Entities
        self._render_ghosts()
        self._render_bot()
        
        # Debug
        if self.show_debug: self._render_debug()
        
        # UI
        self._render_ui()
        
        pygame.display.flip()

    def _render_danger(self):
        dmap = self.engine.tracker.get_danger_map(0)
        if dmap.size == 0: return
        surf = pygame.Surface((self.engine.maze.width*cs, self.engine.maze.height*cs), pygame.SRCALPHA)
        arr = pygame.surfarray.pixels_alpha(surf)
        # Normalize 0-1 -> 0-180 Alpha
        alpha = (dmap.T * 180).astype(np.uint8) # Transpose for surfarray (W,H)
        arr[:] = alpha
        del arr
        # Tint Red
        red = pygame.Surface(surf.get_size()); red.fill((255, 20, 20))
        red.blit(surf, (0,0), special_flags=pygame.BLEND_RGBA_MULT)
        self.screen.blit(red, (0,0))

    def _render_maze(self):
        m = self.engine.maze
        for y in range(m.height):
            for x in range(m.width):
                r = pygame.Rect(x*cs, y*cs, cs, cs)
                if (x,y) in m.walls:
                    pygame.draw.rect(self.screen, COLORS['wall'], r)
                elif (x,y) in m.pellets and (x,y) not in m.eaten_pellets:
                    col = COLORS['power'] if (x,y) in m.power_pellets else COLORS['pellet']
                    rad = cs//3 if (x,y) in m.power_pellets else cs//6
                    pygame.draw.circle(self.screen, col, r.center, rad)
                elif (x,y) == m.exit_pos:
                    pygame.draw.rect(self.screen, COLORS['exit'], r.inflate(-6,-6), border_radius=4)

    def _render_bot(self):
        b = self.engine.pacbot
        cs = self.cs
        # Trail
        if len(self.engine.path_history) > 1:
            pts = [(int(x*cs), int(y*cs)) for x,y in self.engine.path_history]
            pygame.draw.lines(self.screen, COLORS['trail'][:3], False, pts, 2)
        
        # Body Triangle
        px, py = b.pose.x * cs, b.pose.y * cs
        size = cs * 0.45
        th = b.pose.theta
        p1 = (px + size*np.cos(th), py + size*np.sin(th))
        p2 = (px + size*0.6*np.cos(th+2.5), py + size*0.6*np.sin(th+2.5))
        p3 = (px + size*0.6*np.cos(th-2.5), py + size*0.6*np.sin(th-2.5))
        pygame.draw.polygon(self.screen, COLORS['bot'], [p1,p2,p3])
        
        # Belief Crosshair
        bp = b.believed_pose
        bx, by = bp.x*cs, bp.y*cs
        pygame.draw.line(self.screen, (0,255,255), (bx-6,by), (bx+6,by), 1)
        pygame.draw.line(self.screen, (0,255,255), (bx,by-6), (bx,by+6), 1)

        # Path
        if b.path:
            pts = [((p[0]+0.5)*cs, (p[1]+0.5)*cs) for p in b.path]
            pygame.draw.lines(self.screen, COLORS['path'], False, pts, 2)
            if b.target:
                tx, ty = (b.target[0]+0.5)*cs, (b.target[1]+0.5)*cs
                pygame.draw.circle(self.screen, COLORS['target'], (int(tx),int(ty)), 10, 2)

    def _render_ghosts(self):
        cs = self.cs
        for g in self.engine.tracker.ghosts.values():
            gx, gy = g.pose.x*cs, g.pose.y*cs
            pygame.draw.circle(self.screen, COLORS['ghost'], (int(gx),int(gy)), cs//2-2)
            # Dir
            dx, dy = (cs//2)*np.cos(g.pose.theta), (cs//2)*np.sin(g.pose.theta)
            pygame.draw.line(self.screen, (255,255,255), (gx,gy), (gx+dx,gy+dy), 2)
            
            # Prediction
            if self.show_debug and g.predicted_traj:
                pts = [(int(p[0]*cs), int(p[1]*cs)) for p in g.predicted_traj[::3]]
                if len(pts)>1: pygame.draw.lines(self.screen, (255,150,150,100), False, pts, 1)
            
            # Label
            lbl = self.font.render(g.state.name, True, (200,200,200))
            self.screen.blit(lbl, (gx-lbl.get_width()//2, gy - cs//2 - 14))

    def _render_debug(self):
        pass # Sensor rays etc.

    def _render_ui(self):
        y0 = self.engine.maze.height * self.cs + 5
        pygame.draw.rect(self.screen, COLORS['ui_bg'], (0, y0, self.w, 115))
        e = self.engine; b = e.pacbot
        tr = max(0, CFG.MAX_TIME - e.sim_time)
        
        if e.game_over:
            msg = "SUCCESS: EXIT REACHED" if e.win else f"FAILURE: {e.reason}"
            col = (0,255,0) if e.win else (255,60,60)
            surf = self.bfont.render(msg, True, col)
            self.screen.blit(surf, (self.w//2 - surf.get_width()//2, y0 + 20))
        else:
            lines = [
                f"TIME: {tr:5.1f}s / {CFG.MAX_TIME}s  |  SCORE: {b.score:6d}  |  PELLETS: {len(e.maze.eaten_pellets):3d}/{len(e.maze.pellets):3d}",
                f"STATE: {b.state.name:12s}  |  AGGRO: {e.decision.aggression:.2f}  |  VEL: {b.velocity:.2f}  |  POS: ({b.pose.x:5.2f},{b.pose.y:5.2f})",
                f"BELIEF: ({b.believed_pose.x:5.2f},{b.believed_pose.y:5.2f})  |  GHOSTS: {len(e.tracker.ghosts)}  |  PATH LEN: {len(b.path)}",
                f"[SPACE] Pause  [S] Step  [R] Reset  [V] Debug  [D] Danger  [1-4] Strategy"
            ]
            for i, t in enumerate(lines):
                self.screen.blit(self.font.render(t, True, COLORS['text']), (10, y0 + 5 + i*22))
