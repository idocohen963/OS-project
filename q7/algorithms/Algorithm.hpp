#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include "../Graph.hpp"
#include <string>

/**
 * @class Algorithm
 * @brief Abstract base class for graph algorithms.
 * 
 * This is the base interface that all graph algorithms must implement.
 * It follows the Strategy design pattern, allowing different algorithms
 * to be used interchangeably through a common interface.
 * 
 */
class Algorithm {
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes.
     */
    virtual ~Algorithm() {}
    
    /**
     * @brief Executes the algorithm on the given graph.
     * @param graph The graph to run the algorithm on
     * @return A string containing the results of the algorithm execution
     * 
     * This is a pure virtual function that must be implemented by all
     * derived algorithm classes. Each algorithm should return results
     * in a human-readable string format.
     */
    virtual std::string run(const Graph::Graph& graph) = 0;
};

#endif // ALGORITHM_HPP