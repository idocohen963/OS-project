#ifndef MAX_FLOW_HPP
#define MAX_FLOW_HPP

#include "Algorithm.hpp"
#include <vector>
#include <string>

/**
 * @class MaxFlow
 * @brief Maximum Flow algorithm implementation using Ford-Fulkerson method with BFS.
 * 
 * This class implements the Ford-Fulkerson algorithm using BFS (Edmonds-Karp algorithm)
 * to find the maximum flow in a graph
 * 
 * The algorithm finds the maximum flow from source vertex 0 to sink vertex n-1.
 * It uses BFS to find augmenting paths and updates the residual graph until no
 * more augmenting paths exist.
 */
class MaxFlow : public Algorithm {
public:
    /**
     * @brief Executes the Ford-Fulkerson algorithm with BFS to find maximum flow.
     * @param graph The flow network (directed graph with capacities as edge weights)
     * @return A string containing the maximum flow value from vertex 0 to vertex n-1
     * 
     * The algorithm treats edge weights as capacities and finds the maximum flow
     * from source (vertex 0) to sink (vertex n-1). If the graph has fewer than
     * 2 vertices, the maximum flow is 0.
     * 
     */
    std::string run(const Graph::Graph& graph) override;
};

#endif // MAX_FLOW_HPP