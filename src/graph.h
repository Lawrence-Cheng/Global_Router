#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <limits>

struct Edge {
    int to;
    int baseCost; 
};

class Graph {
public:
    Graph();
    explicit Graph(int numVertices);
    void resize(int numVertices);
    int numVertices() const { return static_cast<int>(adj_.size()); }
    void addEdge(int u, int v, int baseCost);
    const std::vector<Edge>& adj(int u) const { return adj_[u]; }

private:
    std::vector<std::vector<Edge>> adj_;
};

// CHANGED: Return type is now void for performance optimization
void dijkstra(
    const Graph &g,
    int source,
    int target,
    const std::vector<long long> &vertexCosts,
    const std::vector<int> &h_vals,
    std::vector<int> *outPrev = nullptr
);

const long long INF = std::numeric_limits<long long>::max() / 2;

#endif // GRAPH_H