#pragma once
#include "types.hpp"

namespace pacbot {

// Main Planning Function
// danger_map: Optional pointer to 2D vector (H x W) of danger costs [0, inf)
// danger_weight: Multiplier for danger cost
PathResult astar_plan(
    const GridMap& grid, 
    const std::array<int, 2>& start, 
    const std::array<int, 2>& goal,
    const DangerMap* danger_map = nullptr,
    double danger_weight = 10.0,
    int max_expansions = 10000 // Safety limit
);

// Heuristics
inline double heuristic_manhattan(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}
inline double heuristic_octile(int x1, int y1, int x2, int y2) {
    int dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
    return (dx + dy) + (M_SQRT2 - 2) * std::min(dx, dy); // For 8-dir
}

} // namespace pacbot
