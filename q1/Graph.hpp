#pragma once
#include <vector>


namespace Graph{

/**
 * @class Graph
 * @brief Represents a directed or undirected graph with weighted edges.
 */
class Graph{
private:
    bool directed; ///< Whether the graph is directed
    int numVertices; ///< Number of vertices in the graph
    std::vector<std::vector<std::pair<int, int>>> adj; ///< Adjacency list: for each vertex, a list of pairs (destination vertex, weight)

public:
    /**
     * @brief Constructs a graph with a given number of vertices.
     * @param V Number of vertices
     * @param isDirected Whether the graph is directed (default: false)
     */
    Graph(int V, bool isDirected = false);

    /**
     * @brief Destructor for the graph.
     */
    ~Graph();

    /**
     * @brief Adds an edge from src to dest with a given weight (default 1).
     * @param src Source vertex
     * @param dest Destination vertex
     * @param weight Edge weight (default 1)
     * @throws std::out_of_range if src or dest are invalid vertex indices
     * @throws std::invalid_argument if edge already exists or if attempting to create a self-loop
     */
    void addEdge(int src, int dest, int weight = 1);

    /**
     * @brief Checks if there is an edge from src to dest.
     * @param src Source vertex
     * @param dest Destination vertex
     * @return true if the edge exists, false otherwise
     */
    bool hasEdge(int src, int dest) const;

    /**
     * @brief Returns the number of vertices in the graph.
     * @return Number of vertices
     */
    int getNumVertices() const;

    /**
     * @brief Checks if the graph is directed.
     * @return true if the graph is directed, false otherwise
     */
    bool isDirected() const;

    /**
     * @brief Returns the weight of the edge from src to dest.
     * @param src Source vertex
     * @param dest Destination vertex
     * @return The edge weight, or a suitable value if the edge does not exist
     */
    int getEdgeWeight(int src, int dest) const;
    /**
     * @brief Prints the graph to stdout in adjacency-list format.
     * For undirected graphs, both directions are present in the adjacency lists.
     */
    void printGraph() const;
};
}