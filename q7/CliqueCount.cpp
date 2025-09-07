#include "CliqueCount.hpp"
#include <vector>
#include <string>

/**
 * @brief Counts all cliques of size 2-6 in the graph.
 * @param graph The graph to analyze for cliques
 * @return A string containing the total number of cliques found
 * 
 * Brute Force Clique Counting Algorithm:
 * 1. Use bit manipulation to generate all possible subsets of vertices
 * 2. Filter subsets to only consider those of size 2-6
 * 3. For each valid subset, check if it forms a clique
 * 4. A subset is a clique if every pair of vertices has an edge
 * 

 */
std::string CliqueCount::run(const Graph::Graph& graph) {
    int n = graph.getNumVertices();
    int count = 0;
    
    // Enumerate all possible subsets using bit manipulation
    // mask represents a subset: bit i is 1 if vertex i is in the subset
    for (int mask = 1; mask < (1 << n); ++mask) {
        std::vector<int> nodes;  // Store vertices in current subset
        
        // Extract vertices from the bit mask
        for (int i = 0; i < n; ++i) {
            if (mask & (1 << i)) {  // Check if bit i is set
                nodes.push_back(i); // Add vertex i to subset
            }
        }
        
        // Only consider subsets of size 2-6 (clique size constraints)
        if (nodes.size() < 2 || nodes.size() > 6) continue;
        
        bool isClique = true;  // Assume subset forms a clique until proven otherwise
        
        // Check if every pair of vertices in the subset has an edge
        for (size_t i = 0; i < nodes.size(); ++i) {
            for (size_t j = i + 1; j < nodes.size(); ++j) {
                bool hasForwardEdge = graph.hasEdge(nodes[i], nodes[j]);
                bool hasReverseEdge = graph.hasEdge(nodes[j], nodes[i]);
                
                // For undirected graphs: need edge in at least one direction (stored in both)
                // For directed graphs: need edges in both directions for a complete subgraph
                if (graph.isDirected()) {
                    if (!hasForwardEdge || !hasReverseEdge) {
                        isClique = false;
                        break;
                    }
                } else {
                    if (!hasForwardEdge) {
                        isClique = false;
                        break;
                    }
                }
            }
            if (!isClique) break;
        }
        
        // If all pairs are connected, we found a clique
        if (isClique) {
            ++count;
        }
    }
    
    return "Number of cliques (size 2-6): " + std::to_string(count);
}