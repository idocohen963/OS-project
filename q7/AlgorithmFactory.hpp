#ifndef ALGORITHM_FACTORY_HPP
#define ALGORITHM_FACTORY_HPP

#include "SCC.hpp"
#include "MST.hpp"
#include "MaxFlow.hpp"
#include "Algorithm.hpp"
#include "CliqueCount.hpp"
#include <string>

/**
 * @class GraphAlgorithmFactory
 * @brief Factory class for creating graph algorithm instances.
 * 
 * This factory implements the Factory design pattern to create instances
 * of different graph algorithms based on string identifiers. It centralizes
 * algorithm creation and provides a clean interface for algorithm selection.
 * 
 * Supported algorithms:
 * - "scc": Strongly Connected Components algorithm (Kosaraju's algorithm)
 * - "maxflow": Maximum Flow algorithm (Ford-Fulkerson with BFS)
 * - "mst": Minimum Spanning Tree algorithm (Prim's algorithm)
 * - "clique": Clique counting algorithm (brute force enumeration)
 * 
 */
class GraphAlgorithmFactory {
public:
    /**
     * @brief Creates an algorithm instance based on the given name.
     * @param name The name identifier of the algorithm to create
     * @return A pointer to the created Algorithm instance, or nullptr if the name is not recognized
     * 
     */
    static Algorithm* create(const std::string& name) {
        if (name == "scc") return new SCC();           // Strongly Connected Components
        if (name == "maxflow") return new MaxFlow();   // Maximum Flow
        if (name == "mst") return new MST();           // Minimum Spanning Tree
        if (name == "clique") return new CliqueCount(); // Clique Count
        return nullptr;  // Unknown algorithm name
    }
};

#endif // ALGORITHM_FACTORY_HPP