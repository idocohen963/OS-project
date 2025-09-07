#ifndef CLIQUE_COUNT_HPP
#define CLIQUE_COUNT_HPP

#include "Algorithm.hpp"

/**
 * @class CliqueCount
 * @brief Clique counting algorithm using brute force enumeration.
 * 
 * This class implements a brute force algorithm to count all cliques of size 2-6
 * in a graph. A clique is a subset of vertices where every two distinct vertices
 * are adjacent (forming a complete subgraph).
 * 
 * The algorithm uses bit manipulation to enumerate all possible subsets of vertices
 * and checks if each subset forms a clique by verifying that all pairs of vertices
 * in the subset are connected by edges.
 * 
 */
class CliqueCount : public Algorithm {
public:
    /**
     * @brief Counts all cliques of size 2-5 in the graph using brute force.
     * @param graph The graph to analyze (works on both directed and undirected graphs)
     * @return A string containing the total count of cliques found
     * 
     * The algorithm:
     * 1. Enumerates all possible subsets of vertices using bit manipulation
     * 2. For each subset of size 2-5, checks if it forms a clique
     * 3. For undirected graphs: a subset is a clique if every pair has an edge
     * 4. For directed graphs: a subset is a clique if every pair has edges in both directions
     * 5. Returns the total count of valid cliques
     * 
     * @note For directed graphs, requires bidirectional edges for clique membership
     * @note The size range (2-6) is hardcoded but can be easily modified
     */
    std::string run(const Graph::Graph& graph) override;
};

#endif // CLIQUE_COUNT_HPP