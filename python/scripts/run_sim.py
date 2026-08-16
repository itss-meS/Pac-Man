#!/usr/bin/env python3
"""Main Simulation Entry Point"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pacbot.sim.engine import SimulationEngine
from pacbot.sim.visualizer import Visualizer
from pacbot.utils.config import CFG
import pygame
import time

def main():
    print("="*60)
    print("PACBOT COMPETITION SIMULATION - PHASE 1")
    print("="*60)
    print(f"Config: {CFG.MAZE_WIDTH}x{CFG.MAZE_HEIGHT} | Max Time: {CFG.MAX_TIME}s | DT: {CFG.SIM_DT}s")
    print("Controls: [SPACE] Pause | [S] Step | [R] Reset | [V] Debug | [D] Danger | [1-4] Strategy")
    print("="*60)

    # Build C++ Core if needed (Hint)
    try:
        import pacbot_core
        print("[OK] C++ Core (pacbot_core) loaded.")
    except ImportError:
        print("[WARN] C++ Core NOT FOUND. Running Pure Python (Slow).")
        print("       Build it: cd ../../cpp_core && cmake -B build && cmake --build build")

    engine = SimulationEngine() # Random Maze
    vis = Visualizer(engine)

    last = time.perf_counter()
    acc = 0.0
    running = True
    while running:
        now = time.perf_counter()
        frame_dt = now - last
        last = now
        
        running = vis.handle_events()
        if not running: break

        acc += frame_dt
        while acc >= CFG.SIM_DT:
            engine.step(CFG.SIM_DT)
            acc -= CFG.SIM_DT
        
        vis.render()
        vis.clock.tick(CFG.FPS)

    pygame.quit()

if __name__ == "__main__":
    main()
