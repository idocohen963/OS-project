#include "SCC.hpp"
#include <string>
#include <vector>

/**
 * @brief Performs depth-first search on the original graph to determine finishing times.
 * @param g The graph to traverse
 * @param v The current vertex being visited
 * @param visited Vector tracking which vertices have been visited
 * @param order Vector storing vertices in post-order (finishing time order)
 * 
 * This is the first phase of Kosaraju's algorithm. It performs a standard DFS
 * but records vertices in post-order traversal, which gives us the finishing
 * times needed for the second phase.
 */
void SCC::dfs(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& order) const {
    visited[v] = true;  // Mark current vertex as visited
    
    // Visit all unvisited neighbors in the original graph
    for (int u = 0; u < g.getNumVertices(); ++u) {
        if (g.hasEdge(v, u) && !visited[u]) {
            dfs(g, u, visited, order);  // Recursively visit neighbor
        }
    }
    
    // Add vertex to order after visiting all its descendants (post-order)
    order.push_back(v);
}

/**
 * @brief Performs depth-first search on the transpose graph.
 * @param g The original graph (edges traversed in reverse)
 * @param v The current vertex being visited
 * @param visited Vector tracking which vertices have been visited
 * @param component Vector storing vertices in the current SCC
 * 
 * This is the second phase of Kosaraju's algorithm. It performs DFS on the
 * transpose graph (following edges in reverse direction) to collect all
 * vertices in the current strongly connected component.
 */
void SCC::dfsRev(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& component) const {
    visited[v] = true;      // Mark current vertex as visited
    component.push_back(v); // Add vertex to current component
    
    // Visit all unvisited vertices that have edges TO v (transpose graph traversal)
    for (int u = 0; u < g.getNumVertices(); ++u) {
        if (g.hasEdge(u, v) && !visited[u]) {  // Check reverse edge (u -> v becomes v <- u)
            dfsRev(g, u, visited, component);   // Recursively visit in transpose
        }
    }
}

/**
 * @brief Executes Kosaraju's algorithm to find all strongly connected components.
 * @param graph The directed graph to analyze
 * @return A formatted string listing all strongly connected components
 * 
 * Kosaraju's Algorithm:
 * 1. Perform DFS on original graph, recording finishing times
 * 2. Consider vertices in decreasing order of finishing times
 * 3. For each unvisited vertex, perform DFS on transpose graph
 * 4. Each DFS tree in step 3 is a strongly connected component
 */
std::string SCC::run(const Graph::Graph& graph) {
    int n = graph.getNumVertices();
    std::vector<bool> visited(n, false);
    std::vector<int> order;
    
    // Phase 1: DFS on original graph to get finishing times
    for (int i = 0; i < n; ++i) {
        if (!visited[i]) {
            dfs(graph, i, visited, order);  // Fill order with post-order traversal
        }
    }
    
    // Reset visited array for second phase
    std::fill(visited.begin(), visited.end(), false);
    
    int sccCount = 0;
    std::vector<std::vector<int>> sccs;  // Store all strongly connected components
    
    // Phase 2: DFS on transpose graph in reverse finishing time order
    for (int i = n - 1; i >= 0; --i) {
        int v = order[i];  // Process vertices in decreasing finishing time order
        if (!visited[v]) {
            std::vector<int> comp;
            dfsRev(graph, v, visited, comp);  // Find SCC starting from v
            sccs.push_back(comp);             // Store the component
            ++sccCount;
        }
    }
    
    // Format the results for output
    std::string result = "Strongly connected components:";
    for (size_t i = 0; i < sccs.size(); ++i) {
        result += "\nComponent " + std::to_string(i+1) + ": ";
        for (size_t j = 0; j < sccs[i].size(); ++j) {
            result += std::to_string(sccs[i][j]);
            if (j+1 < sccs[i].size()) result += ", ";
        }
    }
    return result;
}