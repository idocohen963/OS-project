#ifndef ALGORITHM_FACTORY_HPP
#define ALGORITHM_FACTORY_HPP

#include "Algorithm.hpp"
#include "SCC.hpp"
#include "MST.hpp"
#include "MaxFlow.hpp"
#include "CliqueCount.hpp"
#include <string>

/**
 * @class AlgorithmFactory
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
class AlgorithmFactory {
public:
    /**
     * @brief Creates an algorithm instance based on the given name.
     * @param name The name identifier of the algorithm to create
     * @return A pointer to the created Algorithm instance, or nullptr if the name is not recognized
     * 
     */
    static Algorithm* create(const int alg_id) {
        if (alg_id == 1) return new MST();           // Minimum Spanning Tree
        if (alg_id == 2) return new SCC();           // Strongly Connected Components
        if (alg_id == 3) return new MaxFlow();   // Maximum Flow
        if (alg_id == 4) return new CliqueCount(); // Clique Count
        return nullptr;  // Unknown algorithm name
    }
};

#endif // ALGORITHM_FACTORY_HPP