#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

namespace Graph {

/**
 * @brief Constructs a graph with a given number of vertices.
 * @param V Number of vertices
 * @param isDirected Whether the graph is directed (default: false)
 * @throws std::invalid_argument if V is negative or zero
 */
Graph::Graph(int V, bool isDirected) : directed(isDirected), numVertices(V), adj(V) {
    if (V <= 0) {
        throw std::invalid_argument("Number of vertices must be positive");
    }
}

/**
 * @brief Destructor for the graph.
 */
Graph::~Graph() {
    // Vector destructor automatically handles cleanup
}

/**
 * @brief Adds an edge from src to dest with a given weight (default 1).
 * @param src Source vertex
 * @param dest Destination vertex
 * @param weight Edge weight (default 1)
 * @throws std::out_of_range if src or dest are invalid vertex indices
 * @throws std::invalid_argument if edge already exists or if attempting to create a self-loop
 */
void Graph::addEdge(int src, int dest, int weight) {
    // Validate vertex indices
    if (src < 0 || src >= numVertices) {
        throw std::out_of_range("Invalid source vertex");
    }
    if (dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Invalid destination vertex");
    }

    // Prevent self-loops
    if (src == dest) {
        throw std::invalid_argument("Self-loops are not allowed");
    }

    // Check if edge already exists and throw error if it does
    if (hasEdge(src, dest)) {
        throw std::invalid_argument("Edge already exists");
    }

    // Add new edge
    adj[src].push_back(std::make_pair(dest, weight));
    
    // For undirected graphs, add the reverse edge as well
    if (!directed && src != dest) {
        adj[dest].push_back(std::make_pair(src, weight));
    }
}

/**
 * @brief Checks if there is an edge from src to dest.
 * @param src Source vertex
 * @param dest Destination vertex
 * @return true if the edge exists, false otherwise
 * @throws std::out_of_range if src or dest are invalid vertex indices
 */
bool Graph::hasEdge(int src, int dest) const {
    // Validate vertex indices
    if (src < 0 || src >= numVertices) {
        throw std::out_of_range("Invalid source vertex");
    }
    if (dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Invalid destination vertex");
    }

    // Search for the edge in the adjacency list of src
    for (size_t i = 0; i < adj[src].size(); i++) {
        if (adj[src][i].first == dest) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Returns the number of vertices in the graph.
 * @return Number of vertices
 */
int Graph::getNumVertices() const {
    return numVertices;
}

/**
 * @brief Checks if the graph is directed.
 * @return true if the graph is directed, false otherwise
 */
bool Graph::isDirected() const {
    return directed;
}

/**
 * @brief Returns the weight of the edge from src to dest.
 * @param src Source vertex
 * @param dest Destination vertex
 * @return The edge weight, or -1 if the edge does not exist
 * @throws std::out_of_range if src or dest are invalid vertex indices
 */
int Graph::getEdgeWeight(int src, int dest) const {
    // Validate vertex indices
    if (src < 0 || src >= numVertices) {
        throw std::out_of_range("Invalid source vertex");
    }
    if (dest < 0 || dest >= numVertices) {
        throw std::out_of_range("Invalid destination vertex");
    }

    // Search for the edge and return its weight
    for (size_t i = 0; i < adj[src].size(); i++) {
        if (adj[src][i].first == dest) {
            return adj[src][i].second;
        }
    }
    
    // Edge not found
    return -1;
}

/**
 * @brief Prints the graph to stdout in adjacency-list format.
 * For undirected graphs, both directions are present in the adjacency lists.
 */
void Graph::printGraph() const {
    for (int v = 0; v < numVertices; ++v) {
        std::cout << v << " -> ";
        const auto &neighbors = adj[v];
        for (size_t i = 0; i < neighbors.size(); ++i) {
            std::cout << "(" << neighbors[i].first << ", w=" << neighbors[i].second << ")";
            if (i + 1 < neighbors.size()) std::cout << ", ";
        }
        std::cout << std::endl;
    }
}

} // namespace Graph