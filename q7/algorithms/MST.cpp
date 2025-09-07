#include "MST.hpp"
#include <limits.h>
#include <vector>
#include <string>

/**
 * @brief Executes Prim's algorithm to find the minimum spanning tree.
 * @param graph The weighted graph to analyze
 * @return A string with the total MST weight or error message if disconnected
 * 
 * Prim's Algorithm:
 * 1. Start with vertex 0 in the MST
 * 2. Repeatedly add the minimum weight edge connecting MST to non-MST vertex
 * 3. Continue until all vertices are in the MST
 * 
 * For directed graphs, the algorithm considers edges in both directions
 * to effectively treat the graph as undirected.
 */
std::string MST::run(const Graph::Graph& graph) {
    int n = graph.getNumVertices();
    std::vector<bool> inMST(n, false);      // Track which vertices are in MST
    std::vector<int> minEdge(n, INT_MAX);   // Minimum edge weight to reach each vertex
    
    minEdge[0] = 0;    // Start from vertex 0
    int totalWeight = 0;
    
    // Process all vertices
    for (int i = 0; i < n; ++i) {
        int u = -1;
        
        // Find the minimum weight vertex not yet in MST
        for (int v = 0; v < n; ++v) {
            if (!inMST[v] && (u == -1 || minEdge[v] < minEdge[u])) {
                u = v;  // Update minimum vertex
            }
        }
        
        // If no reachable vertex found, graph is disconnected
        if (minEdge[u] == INT_MAX) {
            return "Graph not connected";
        }
        
        inMST[u] = true;           // Add vertex u to MST
        totalWeight += minEdge[u]; // Add edge weight to total
        
        // Update minimum edge weights for remaining vertices
        for (int v = 0; v < n; ++v) {
            if (!inMST[v]) {
                // Check edges in both directions for directed graphs
                int w1 = graph.getEdgeWeight(u, v);  // Edge from u to v
                int w2 = graph.getEdgeWeight(v, u);  // Edge from v to u
                int w = 0;
                
                // Determine the effective edge weight between u and v
                if (w1 > 0 && w2 > 0) {
                    w = std::min(w1, w2);  // Take minimum if both directions exist
                } else if (w1 > 0) {
                    w = w1;  // Only u->v edge exists
                } else if (w2 > 0) {
                    w = w2;  // Only v->u edge exists
                }
                
                // Update minimum edge to vertex v if we found a better edge
                if (w > 0 && w < minEdge[v]) {
                    minEdge[v] = w;
                }
            }
        }
    }
    
    return "MST total weight: " + std::to_string(totalWeight);
}