
// router.cpp
#include "router.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <set>
#include <chrono> 
#include <limits>
#include <random>
#include <vector>

namespace {

// --- FINAL CONFIGURATION ---
constexpr double INITIAL_PRES_FAC = 1.0;   
constexpr double PRES_FAC_GROWTH = 1.1; 
constexpr int HISTORY_INCREMENT = 100; 

// Cap to prevent integer overflow
constexpr long long MAX_COST_CAP = 4000000000000000000LL; 

int totalVertices(const Grid &grid) {
    return grid.numLayers() * grid.xSize() * grid.ySize();
}

// Inline bounds check helper
inline bool isValid(const Grid &grid, int l, int c, int r) {
    return l >= 0 && l < grid.numLayers() &&
           c >= 0 && c < grid.xSize() &&
           r >= 0 && r < grid.ySize();
}

std::vector<int> computeHeuristics(const Grid &grid, int targetIdx, double weight) {
    int total = totalVertices(grid);
    std::vector<int> h(total, 0); 

    if (weight > 0.001) {
        Coord3D tgt = grid.fromIndex(targetIdx);
        int minStep = 1; 
        for(int i=0; i<total; ++i) {
            Coord3D c = grid.fromIndex(i);
            int dist = std::abs(c.col - tgt.col) + std::abs(c.row - tgt.row) + std::abs(c.layer - tgt.layer) * 3; 
            h[i] = static_cast<int>(dist * minStep * weight); 
        }
    }
    return h;
}

std::vector<Coord3D> reconstructPath(const Grid &grid, int sourceIdx, int targetIdx, const std::vector<int> &prev) {
    std::vector<Coord3D> path;
    int curr = targetIdx;
    int total = totalVertices(grid);

    if (prev.size() != (size_t)total) return {};
    if (curr < 0 || curr >= total || sourceIdx < 0 || sourceIdx >= total) return {};
    if (curr != sourceIdx && prev[curr] == -1) return {};
    
    int steps = 0;
    while (curr != -1) {
        if (curr < 0 || curr >= total) break;
        path.push_back(grid.fromIndex(curr));
        if (curr == sourceIdx) break;
        
        curr = prev[curr];
        if (++steps > total) return {}; 
    }
    
    if (path.empty()) return {};
    
    Coord3D pStart = path.back();
    Coord3D gStart = grid.fromIndex(sourceIdx);
    if (pStart.layer != gStart.layer || pStart.col != gStart.col || pStart.row != gStart.row) return {};

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Coord3D> buildFallbackPath(const Grid &grid, const Net &net) {
    std::vector<Coord3D> path;
    Coord3D curr = net.pin1;
    Coord3D target = net.pin2;
    
    curr.layer = std::max(0, std::min(curr.layer, grid.numLayers() - 1));
    curr.col = std::max(0, std::min(curr.col, grid.xSize() - 1));
    curr.row = std::max(0, std::min(curr.row, grid.ySize() - 1));

    path.push_back(curr);
    
    int watchdog = 0;
    while ((curr.col != target.col || curr.row != target.row || curr.layer != target.layer) && watchdog++ < 5000) {
        if (!isValid(grid, curr.layer, curr.col, curr.row)) break;

        const auto& lInfo = grid.layerInfo(curr.layer);
        bool moved = false;

        if (curr.col != target.col && lInfo.direction == 'H') {
            int nextC = (curr.col < target.col) ? curr.col + 1 : curr.col - 1;
            if (isValid(grid, curr.layer, nextC, curr.row)) {
                curr.col = nextC;
                moved = true;
            }
        } 
        else if (curr.row != target.row && lInfo.direction == 'V') {
            int nextR = (curr.row < target.row) ? curr.row + 1 : curr.row - 1;
            if (isValid(grid, curr.layer, curr.col, nextR)) {
                curr.row = nextR;
                moved = true;
            }
        }

        if (!moved) {
            int nextLayer = curr.layer;
            if (curr.layer < target.layer) nextLayer++;
            else if (curr.layer > target.layer) nextLayer--;
            else {
                if (nextLayer + 1 < grid.numLayers()) nextLayer++;
                else nextLayer--;
            }

            if (nextLayer != curr.layer && isValid(grid, nextLayer, curr.col, curr.row)) {
                curr.layer = nextLayer;
            } else {
                break; 
            }
        }
        path.push_back(curr);
    }
    return path;
}

struct GridState {
    std::vector<int> demands;
    std::vector<long long> history;
    std::vector<long long> currentCosts;
    double pres_fac;
    const Grid* gridRef;
    mutable std::mt19937 rng; 

    GridState(const Grid& g) : gridRef(&g), rng(12345) {
        int total = totalVertices(g);
        demands.assign(total, 0);
        history.assign(total, 0);
        currentCosts.assign(total, 0);
        pres_fac = INITIAL_PRES_FAC;
    }

    long long calculateCost(int idx) const {
        Coord3D c = gridRef->fromIndex(idx);
        if (!isValid(*gridRef, c.layer, c.col, c.row)) return MAX_COST_CAP;

        int cap = gridRef->capacity(c.layer, c.col, c.row);
        int d = demands[idx];
        
        long long base = 100;
        long long h_term = history[idx];
        long long noise = (h_term > 20) ? (rng() % (h_term / 20 + 1)) : 0;

        long long intrinsic = base + h_term + noise;
        double multiplier = 1.0;
        
        if (d >= cap) {
            int overflow = d - cap;
            double exp_term = std::pow(2.0, (double)std::min(overflow, 25)); 
            multiplier = 1.0 + (pres_fac * exp_term);
        } else if (d + 1 == cap) {
            multiplier = 1.0 + (pres_fac * 0.1); 
        }

        double total = (double)intrinsic * multiplier;
        if (total >= (double)MAX_COST_CAP) return MAX_COST_CAP;
        return (long long)total;
    }

    void updatePath(const std::vector<Coord3D>& path, int dir) {
        std::set<int> visited;
        int total = totalVertices(*gridRef);
        for (const auto& c : path) {
            if (!isValid(*gridRef, c.layer, c.col, c.row)) continue;

            int idx = gridRef->gcellIndex(c.layer, c.col, c.row);
            if (idx < 0 || idx >= total) continue;

            if (visited.count(idx)) continue;
            visited.insert(idx);
            demands[idx] += dir;
            currentCosts[idx] = calculateCost(idx);
        }
    }

    void refreshAllCosts() {
        int total = demands.size();
        for(int i=0; i<total; ++i) {
            currentCosts[i] = calculateCost(i);
        }
    }
    
    void blastHistory() {
        int total = demands.size();
        int blasted = 0;
        for(int i=0; i<total; ++i) {
            Coord3D c = gridRef->fromIndex(i);
            if (!isValid(*gridRef, c.layer, c.col, c.row)) continue;

            int cap = gridRef->capacity(c.layer, c.col, c.row);
            if (demands[i] > cap) {
                if (history[i] == 0) history[i] = 1000;
                else history[i] *= 10; 
                
                if (history[i] > MAX_COST_CAP / 1000) history[i] = MAX_COST_CAP / 1000;
                
                blasted++;
            }
        }
        refreshAllCosts();
        if(blasted > 0) std::cout << "  [BLAST] Intensified history for " << blasted << " congested cells.\n";
    }
};

void printDetailedCost(Grid &grid, const std::vector<std::vector<Coord3D>> &paths, std::chrono::steady_clock::time_point startTime) {
    grid.resetDemand();
    for (const auto &path : paths) {
        std::set<int> visited;
        for (const auto &c : path) {
            if (!isValid(grid, c.layer, c.col, c.row)) continue;
            int idx = grid.gcellIndex(c.layer, c.col, c.row);
            if (visited.find(idx) == visited.end()) {
                grid.addDemandForNetGCell(0, c.layer, c.col, c.row);
                visited.insert(idx);
            }
        }
    }

    long long totalOverflow = 0;
    int maxOverflow = 0;
    int totalGCells = grid.numLayers() * grid.xSize() * grid.ySize();
    for (int i = 0; i < totalGCells; ++i) {
        int d = grid.demandByIndex(i);
        Coord3D c = grid.fromIndex(i);
        int cap = grid.capacity(c.layer, c.col, c.row);
        int ov = std::max(0, d - cap);
        totalOverflow += ov;
        if (ov > maxOverflow) maxOverflow = ov;
    }

    long long totalWL = 0;
    int totalVias = 0;
    for (const auto &path : paths) {
        if (path.empty()) continue;
        for (size_t k = 1; k < path.size(); ++k) {
            const auto &curr = path[k];
            const auto &prev = path[k-1];
            if (curr.layer != prev.layer) { totalWL += grid.wlViaCost(); totalVias++; }
            else if (curr.col != prev.col) { totalWL += grid.horizontalDist(std::min(curr.col, prev.col)); }
            else if (curr.row != prev.row) { totalWL += grid.verticalDist(std::min(curr.row, prev.row)); }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();

    std::cout << "\n=================================================\n";
    std::cout << "             PERFORMANCE MATRIX                  \n";
    std::cout << "=================================================\n";
    std::cout << std::left << std::setw(20) << "Metric" << " | " << "Value" << "\n";
    std::cout << "-------------------------------------------------\n";
    std::cout << std::left << std::setw(20) << "Overflow" << " | " << totalOverflow << "\n";
    std::cout << std::left << std::setw(20) << "Cost (Wirelen)" << " | " << totalWL << "\n";
    std::cout << std::left << std::setw(20) << "Runtime" << " | " << duration << "s\n";
    std::cout << "-------------------------------------------------\n";
    std::cout << "Max Overflow         : " << maxOverflow << "\n";
    std::cout << "Total Vias           : " << totalVias << "\n";
    std::cout << "=================================================\n\n";
}

} // namespace

Graph buildGraphFromGrid(const Grid &grid) {
    int n = totalVertices(grid);
    Graph g(n);
    for (int l = 0; l < grid.numLayers(); ++l) {
        char dir = grid.layerInfo(l).direction;
        for (int row = 0; row < grid.ySize(); ++row) {
            for (int col = 0; col < grid.xSize(); ++col) {
                int u = grid.gcellIndex(l, col, row);
                if (dir == 'H' && col + 1 < grid.xSize()) {
                    int v = grid.gcellIndex(l, col + 1, row);
                    g.addEdge(u, v, grid.horizontalDist(col));
                    g.addEdge(v, u, grid.horizontalDist(col));
                }
                if (dir == 'V' && row + 1 < grid.ySize()) {
                    int v = grid.gcellIndex(l, col, row + 1);
                    g.addEdge(u, v, grid.verticalDist(row));
                    g.addEdge(v, u, grid.verticalDist(row));
                }
                if (l + 1 < grid.numLayers()) {
                    int v = grid.gcellIndex(l + 1, col, row);
                    g.addEdge(u, v, grid.wlViaCost());
                    g.addEdge(v, u, grid.wlViaCost());
                }
            }
        }
    }
    return g;
}

RoutingResult runRouting(Grid &grid, const std::vector<Net> &nets) {
    RoutingResult result;
    result.nets.resize(nets.size());
    Graph graph = buildGraphFromGrid(grid);
    
    std::vector<std::vector<Coord3D>> currentPaths(nets.size());
    std::vector<std::vector<Coord3D>> bestPaths(nets.size());
    long long bestOverflow = std::numeric_limits<long long>::max();

    std::vector<int> netOrder(nets.size());
    for(size_t i=0; i<nets.size(); ++i) netOrder[i] = i;

    std::sort(netOrder.begin(), netOrder.end(), [&](int a, int b){
        int distA = std::abs(nets[a].pin1.col - nets[a].pin2.col) + std::abs(nets[a].pin1.row - nets[a].pin2.row);
        int distB = std::abs(nets[b].pin1.col - nets[b].pin2.col) + std::abs(nets[b].pin1.row - nets[b].pin2.row);
        return distA > distB; 
    });

    int maxIters = 150; 
    GridState state(grid); 
    
    std::mt19937 rng(1337);
    auto startTime = std::chrono::steady_clock::now();
    long long lastOverflow = -1;
    int stuckCounter = 0;
    int lastImprovedIter = 0; 

    // --- INITIAL ROUTE ---
    std::cout << "Initial Routing...\n";
    for (int netId : netOrder) {
        const Net &net = nets[netId];
        int src = grid.gcellIndex(net.pin1.layer, net.pin1.col, net.pin1.row);
        int dst = grid.gcellIndex(net.pin2.layer, net.pin2.col, net.pin2.row);
        
        std::vector<int> predecessors;
        std::vector<int> h_vals = computeHeuristics(grid, dst, 2.0);
        
        dijkstra(graph, src, dst, state.currentCosts, h_vals, &predecessors);
        
        auto path = reconstructPath(grid, src, dst, predecessors);
        if (path.empty()) path = buildFallbackPath(grid, net);
        currentPaths[netId] = path;
        state.updatePath(path, 1);
    }

    // --- ITERATIVE RIP-UP AND REROUTE ---
    for (int iter = 0; iter < maxIters; ++iter) {
        double currentAStarWeight = 2.0;
        
        if (stuckCounter > 2 || (bestOverflow < 100 && bestOverflow > 0)) {
            currentAStarWeight = 0.0; 
        }

        std::cout << "Iteration " << iter + 1 << " (PresFac: " << state.pres_fac 
                  << ", H-Weight: " << currentAStarWeight << ")...\n";
        
        if (bestOverflow < 200) {
            std::vector<int> netOverCost(nets.size(), 0);
            for(size_t nid=0; nid<nets.size(); ++nid) {
                for(const auto& c : currentPaths[nid]) {
                    if (!isValid(grid, c.layer, c.col, c.row)) continue;
                    int idx = grid.gcellIndex(c.layer, c.col, c.row);
                    int cap = grid.capacity(c.layer, c.col, c.row);
                    if (state.demands[idx] > cap) {
                        netOverCost[nid] += (state.demands[idx] - cap);
                    }
                }
            }
            std::sort(netOrder.begin(), netOrder.end(), [&](int a, int b){
                if (netOverCost[a] != netOverCost[b]) return netOverCost[a] > netOverCost[b];
                return a < b; // Deterministic tie-breaker
            });
        } else {
            std::shuffle(netOrder.begin(), netOrder.end(), rng);
        }

        bool timeLimitExceeded = false;
        int checkCounter = 0;

        for (int netId : netOrder) {
            if (++checkCounter % 100 == 0) {
                auto curr = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curr - startTime).count() > 580) {
                    std::cout << "  -> TIME LIMIT STOP.\n";
                    timeLimitExceeded = true; 
                    break;
                }
            }

            state.updatePath(currentPaths[netId], -1); 

            const Net &net = nets[netId];
            int src = grid.gcellIndex(net.pin1.layer, net.pin1.col, net.pin1.row);
            int dst = grid.gcellIndex(net.pin2.layer, net.pin2.col, net.pin2.row);
            
            std::vector<int> predecessors;
            std::vector<int> h_vals = computeHeuristics(grid, dst, currentAStarWeight);

            dijkstra(graph, src, dst, state.currentCosts, h_vals, &predecessors);

            auto path = reconstructPath(grid, src, dst, predecessors);
            if (path.empty()) path = buildFallbackPath(grid, net);
            currentPaths[netId] = path;

            state.updatePath(currentPaths[netId], 1); 
        }

        if (timeLimitExceeded) break;

        long long currentOverflow = 0;
        int total = state.demands.size();
        for(int i=0; i<total; ++i) {
            Coord3D c = grid.fromIndex(i);
            int d = state.demands[i];
            int cap = grid.capacity(c.layer, c.col, c.row);
            if (d > cap) {
                currentOverflow += (d - cap);
                state.history[i] += HISTORY_INCREMENT; 
            }
        }

        std::cout << "  -> Overflow: " << currentOverflow << "\n";

        if (currentOverflow < bestOverflow) {
            bestOverflow = currentOverflow;
            bestPaths = currentPaths;
            std::cout << "  -> New Best Solution Saved!\n";
            stuckCounter = 0;
            lastImprovedIter = iter; 
        } else if (currentOverflow == lastOverflow) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }
        
        lastOverflow = currentOverflow;

        if (currentOverflow == 0) {
            std::cout << "  -> Converged!\n";
            break;
        }

        // FIX: Check patience against True Improvement
        if (iter > 50 && (iter - lastImprovedIter > 30)) {
            std::cout << "  -> Stagnation detected (No improvement for 30 iters). Stopping early.\n";
            break;
        }

        // Blast history to unclog (DO NOT RESET lastImprovedIter)
        if (stuckCounter >= 10 && bestOverflow < 50) {
             state.blastHistory();
             stuckCounter = 0;
        }

        if (state.pres_fac < 10000.0) {
            state.pres_fac *= PRES_FAC_GROWTH;
        }
        
        state.refreshAllCosts();
    }

    std::cout << "Restoring Best Solution (Overflow: " << bestOverflow << ")...\n";
    printDetailedCost(grid, bestPaths, startTime);

    for (size_t i = 0; i < nets.size(); ++i) {
        result.nets[i].name = nets[i].name;
        const auto &path = bestPaths[i];
        for (size_t k = 1; k < path.size(); ++k) {
            result.nets[i].segments.push_back(Segment{path[k - 1], path[k]});
        }
    }

    return result;
}

bool writeRouteFile(const std::string &filename, const RoutingResult &result) {
    std::ofstream fout(filename);
    if (!fout) return false;
    for (const RoutedNet &net : result.nets) {
        fout << net.name << "\n(\n";
        for (const Segment &seg : net.segments) {
            fout << seg.from.layer << " " << seg.from.col << " " << seg.from.row << " "
                 << seg.to.layer << " " << seg.to.col << " " << seg.to.row << "\n";
        }
        fout << ")\n";
    }
    return true;
}




