#!/usr/bin/env python3
"""Stress Test: 1000 Random Mazes, Measure Win Rate & Perf"""
import sys, os, time, statistics
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pacbot.sim.engine import SimulationEngine
from pacbot.utils.maze_generator import MazeGenerator
from pacbot.utils.config import CFG

def run_benchmark(num_maps=200, max_steps=50000):
    print(f"[BENCH] Running {num_maps} maps...")
    wins = 0; scores = []; times = []; plan_times = []
    
    for i in range(num_maps):
        if i % 20 == 0: print(f"  Map {i}/{num_maps}...")
        maze = MazeGenerator.generate(seed=1000+i)
        engine = SimulationEngine(maze=maze)
        
        start = time.perf_counter()
        step_count = 0
        while not engine.game_over and step_count < max_steps:
            t0 = time.perf_counter()
            engine.step(CFG.SIM_DT)
            plan_times.append((time.perf_counter() - t0)*1000)
            step_count += 1
        
        times.append(engine.sim_time)
        scores.append(engine.final_score)
        if engine.win: wins += 1
    
    print("\n" + "="*40)
    print(f"RESULTS ({num_maps} Maps)")
    print(f"Win Rate: {wins}/{num_maps} ({100*wins/num_maps:.1f}%)")
    print(f"Avg Score: {statistics.mean(scores):.1f} | Max: {max(scores)}")
    print(f"Avg Sim Time: {statistics.mean(times):.1f}s")
    print(f"Avg Step Time: {statistics.mean(plan_times):.3f}ms | Max: {max(plan_times):.3f}ms")
    if len(plan_times) > 10: print(f"P99 Step Time: {sorted(plan_times)[int(0.99*len(plan_times))]:.3f}ms")
    print("="*40)

if __name__ == "__main__":
    run_benchmark(200)
