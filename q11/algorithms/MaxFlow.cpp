#include "MaxFlow.hpp"
#include <limits.h>
#include <queue>
#include <vector>
#include <string>

/**
 * @brief Executes Ford-Fulkerson algorithm with BFS to find maximum flow.
 * @param graph The flow network where edge weights represent capacities
 * @return A string containing the maximum flow from vertex 0 to vertex n-1
 * 
 * Ford-Fulkerson Algorithm with BFS (Edmonds-Karp):
 * 1. Build capacity matrix from graph edge weights
 * 2. While there exists an augmenting path from source to sink:
 *    a. Find path using BFS (shortest path in terms of hops)
 *    b. Determine bottleneck capacity along the path
 *    c. Update residual capacities along the path
 * 3. Return total flow accumulated
 */
std::string MaxFlow::run(const Graph::Graph& graph) {
    int n = graph.getNumVertices();
    
    // Build capacity matrix from graph edge weights
    std::vector<std::vector<int>> capacity(n, std::vector<int>(n, 0));
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            capacity[u][v] = graph.getEdgeWeight(u, v);  // Edge weights are capacities
        }
    }
    
    int flow = 0;              // Total flow accumulated
    std::vector<int> parent(n); // Parent array for BFS path reconstruction
    
    // BFS lambda function to find augmenting path and return bottleneck capacity
    auto bfs = [&](int s, int t) -> int {
        std::fill(parent.begin(), parent.end(), -1);  // Reset parent array
        parent[s] = -2;  // Mark source as visited with special value
        
        std::queue<std::pair<int, int>> q;  // Queue stores (vertex, min_capacity_to_reach)
        q.push({s, INT_MAX});  // Start from source with infinite capacity
        
        while (!q.empty()) {
            int u = q.front().first;   // Current vertex
            int f = q.front().second;  // Minimum capacity along path to u
            q.pop();
            
            // Explore all adjacent vertices
            for (int v = 0; v < n; ++v) {
                // If vertex v is unvisited and there's available capacity
                if (parent[v] == -1 && capacity[u][v] > 0) {
                    parent[v] = u;  // Set parent for path reconstruction
                    int new_f = std::min(f, capacity[u][v]);  // Update bottleneck capacity
                    
                    if (v == t) return new_f;  // Reached sink, return bottleneck
                    q.push({v, new_f});        // Continue BFS from v
                }
            }
        }
        return 0;  // No augmenting path found
    };
    
    int s = 0, t = n - 1;  // Source is vertex 0, sink is vertex n-1
    int new_flow;
    
    // Main Ford-Fulkerson loop: find augmenting paths until none exist
    do {
        new_flow = bfs(s, t);  // Find augmenting path and its bottleneck capacity
        
        if (new_flow) {
            flow += new_flow;  // Add bottleneck to total flow
            
            // Update residual capacities along the augmenting path
            int v = t;
            while (v != s) {
                int u = parent[v];
                capacity[u][v] -= new_flow;  // Reduce forward capacity
                capacity[v][u] += new_flow;  // Increase backward capacity (for future paths)
                v = u;  // Move backwards along path
            }
        }
    } while (new_flow);  // Continue until no more augmenting paths
    
    return "Max flow from 0 to n-1: " + std::to_string(flow);
}