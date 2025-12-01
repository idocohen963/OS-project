#ifndef MST_HPP
#define MST_HPP

#include "Algorithm.hpp"

/**
 * @class MST
 * @brief Minimum Spanning Tree algorithm implementation using Prim's algorithm.
 * 
 * This class implements Prim's algorithm to find the minimum spanning tree (MST)
 * of a weighted graph. A minimum spanning tree is a subset of edges that connects
 * all vertices with the minimum possible total edge weight, without forming cycles.
 * 
 * The algorithm works on both directed and undirected graphs, but treats directed
 * graphs as undirected by considering edges in both directions.
 */
class MST : public Algorithm {
public:
    /**
     * @brief Executes Prim's algorithm to find the minimum spanning tree.
     * @param graph The weighted graph to find MST for (must be connected)
     * @return A string containing the total weight of the MST, or an error message if disconnected
     * 
     * The algorithm starts from vertex 0 and greedily adds the minimum weight edge
     * that connects a vertex in the MST to a vertex outside the MST, until all
     * vertices are included.
     * 
     * @note Returns "Graph not connected" if the graph is disconnected
     * @note For directed graphs, considers edges in both directions
     */
    std::string run(const Graph::Graph& graph) override;
};

#endif // MST_HPP