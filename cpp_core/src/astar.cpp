#include "pacbot_core/astar.hpp"
#include <queue>
#include <algorithm>
#include <cstring> // memcpy

namespace pacbot {

PathResult astar_plan(const GridMap& grid, const std::array<int, 2>& start, 
                      const std::array<int, 2>& goal, const DangerMap* danger_map, 
                      double danger_weight, int max_expansions) 
{
    PathResult result;
    int H = grid.size();
    if (H == 0) return result;
    int W = grid[0].size();
    
    // Bounds Check
    auto in_bounds = [&](int x, int y){ return x >= 0 && x < W && y >= 0 && y < H; };
    if (!in_bounds(start[0], start[1]) || !in_bounds(goal[0], goal[1])) return result;
    if (grid[start[1]][start[0]] != 0 || grid[goal[1]][goal[0]] != 0) return result;

    // Pre-allocated Data Structures (Reused per call ideally, but stack alloc ok for <100x100)
    std::vector<std::vector<double>> g_score(H, std::vector<double>(W, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<std::array<int,2>>> parent(H, std::vector<std::array<int,2>>(W, {-1,-1}));
    std::vector<std::vector<char>> closed(H, std::vector<char>(W, 0));
    
    // Min-Heap
    std::priority_queue<PathNode, std::vector<PathNode>, std::greater<PathNode>> open;
    
    auto push_node = [&](int x, int y, double g, int px, int py) {
        double h = heuristic_manhattan(x, y, goal[0], goal[1]);
        double danger = 0.0;
        if (danger_map && y < (int)danger_map->size() && x < (int)(*danger_map)[0].size()) {
            danger = (*danger_map)[y][x] * danger_weight;
        }
        open.push({x, y, g, h, g + h + danger, px, py});
    };

    g_score[start[1]][start[0]] = 0.0;
    push_node(start[0], start[1], 0.0, -1, -1);

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    int expansions = 0;

    while (!open.empty() && expansions < max_expansions) {
        auto cur = open.top(); open.pop();
        
        if (closed[cur.y][cur.x]) continue;
        closed[cur.y][cur.x] = 1;
        parent[cur.y][cur.x] = {cur.px, cur.py};
        expansions++;

        if (cur.x == goal[0] && cur.y == goal[1]) {
            result.success = true;
            result.cost = cur.g;
            result.expansions = expansions;
            
            // Reconstruct
            int cx = cur.x, cy = cur.y;
            while (cx != -1) {
                result.path.push_back({cx, cy});
                auto p = parent[cy][cx];
                cx = p[0]; cy = p[1];
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (!in_bounds(nx, ny)) continue;
            if (grid[ny][nx] != 0) continue; // Wall
            if (closed[ny][nx]) continue;

            double ng = cur.g + 1.0; // Uniform cost
            if (g_score[ny][nx] > ng) {
                g_score[ny][nx] = ng;
                push_node(nx, ny, ng, cur.x, cur.y);
            }
        }
    }
    result.expansions = expansions;
    return result; // Failure
}

} // namespace pacbot
