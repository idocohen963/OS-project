#ifndef SCC_HPP
#define SCC_HPP

#include "Algorithm.hpp"
#include <vector>
#include <string>

/**
 * @class SCC
 * @brief Strongly Connected Components algorithm implementation using Kosaraju's algorithm.
 * 
 * This class implements Kosaraju's algorithm to find all strongly connected components
 * in a directed graph. A strongly connected component is a maximal set of vertices
 * such that for every pair of vertices u and v, there is a directed path from u to v
 * and a directed path from v to u.
 * 
 */
class SCC : public Algorithm {
public:
    /**
     * @brief Executes Kosaraju's algorithm to find strongly connected components.
     * @param graph The directed graph to analyze (undirected graphs are treated as directed)
     * @return A string listing all strongly connected components found
     * 
     * The algorithm works in two phases:
     * 1. Perform DFS on the original graph to get finishing times
     * 2. Perform DFS on the transpose graph in decreasing order of finishing times
     */
    std::string run(const Graph::Graph& graph) override;

private:
    /**
     * @brief Performs depth-first search on the original graph.
     * @param g The graph to traverse
     * @param v The current vertex being visited
     * @param visited Vector tracking which vertices have been visited
     * @param order Vector storing vertices in order of finishing times (post-order)
     * 
     * This DFS fills the order vector with vertices in post-order traversal,
     * which gives us the finishing times needed for Kosaraju's algorithm.
     */
    void dfs(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& order) const;
    
    /**
     * @brief Performs depth-first search on the transpose graph.
     * @param g The original graph (edges are traversed in reverse during search)
     * @param v The current vertex being visited
     * @param visited Vector tracking which vertices have been visited
     * @param component Vector storing vertices in the current strongly connected component
     * 
     * This DFS explores the transpose graph by following edges in reverse direction,
     * collecting all vertices reachable from the starting vertex into a component.
     */
    void dfsRev(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& component) const;
};

#endif // SCC_HPP