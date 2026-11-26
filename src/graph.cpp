// src/graph.cpp
#include "graph.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <vector>

// --- Graph Class Implementation ---

Graph::Graph() {
    // Default constructor
}

Graph::Graph(int numVertices) {
    resize(numVertices);
}

void Graph::resize(int numVertices) {
    // reserve 預留空間可稍微提升建構速度
    adj_.clear();
    adj_.resize(numVertices);
    for(auto& vec : adj_) {
        vec.reserve(4); // Grid graph 每個點通常只有 2-6 個鄰居
    }
}

void Graph::addEdge(int u, int v, int baseCost) {
    // 移除邊界檢查以換取極致速度 (呼叫端 router.cpp 已經保證安全)
    adj_[u].push_back({v, baseCost});
}

// --- High Performance Dijkstra / A* Implementation ---

// Priority Queue Element: <F-Score, Vertex Index>
using PII = std::pair<long long, int>;

void dijkstra(
    const Graph &g,
    int source,
    int target,
    const std::vector<long long> &vertexCosts,
    const std::vector<int> &h_vals,
    std::vector<int> *outPrev
) {
    int n = g.numVertices();
    
    // OPTIMIZATION 1: Memory Reuse
    // 使用 static thread_local 避免每次呼叫都重新分配記憶體
    // 這在反覆呼叫數萬次的 Routing 中是巨大的效能提升
    static thread_local std::vector<long long> dist;
    
    if (dist.size() != (size_t)n) {
        dist.resize(n);
    }
    
    // 快速重置 dist 為 INF
    // 這裡不使用 assign，因為 assign 會改變 size，fill 比較快
    std::fill(dist.begin(), dist.end(), INF);

    // Reset predecessors array if provided
    if (outPrev) {
        outPrev->assign(n, -1);
    }

    // Min-Priority Queue
    // 雖然 std::priority_queue 每次都會重新分配，但這是演算法核心無法避免
    std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;

    // Initialize Source
    dist[source] = vertexCosts[source];
    
    // OPTIMIZATION 2: Branch Splitting
    // 將 A* 和 純 Dijkstra 拆開，避免在最內層迴圈做 if (h_vals.empty()) 判斷
    
    if (h_vals.empty()) {
        // --- FAST PATH: Pure Dijkstra (No Heuristic calculation) ---
        pq.push({dist[source], source});

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();

            long long d_u = top.first;
            int u = top.second;

            if (d_u > dist[u]) continue;
            if (u == target) return; // Found target

            // 使用參考 (reference) 避免拷貝 adj list
            const auto& neighbors = g.adj(u);
            for (const auto &edge : neighbors) {
                int v = edge.to;
                // Pre-calculate new distance
                long long new_dist = d_u + edge.baseCost + vertexCosts[v];

                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    if (outPrev) (*outPrev)[v] = u;
                    pq.push({new_dist, v});
                }
            }
        }
    } else {
        // --- A* PATH: With Heuristic ---
        pq.push({dist[source] + h_vals[source], source});

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();

            long long f_val = top.first; // f = g + h
            int u = top.second;

            // Recover g-score
            long long g_val = f_val - h_vals[u];

            if (g_val > dist[u]) continue;
            if (u == target) return;

            const auto& neighbors = g.adj(u);
            for (const auto &edge : neighbors) {
                int v = edge.to;
                
                // Relaxation
                long long new_dist = dist[u] + edge.baseCost + vertexCosts[v];

                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    if (outPrev) (*outPrev)[v] = u;
                    // Push F-score directly
                    pq.push({new_dist + h_vals[v], v});
                }
            }
        }
    }
}