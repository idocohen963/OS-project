#pragma once
#include <vector>
#include <string>


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
    
    /**
     * @brief get the graph in adjacency-list format.
     * For undirected graphs, both directions are present in the adjacency lists.
     * @return the graph in adjacency-list format
     */
    std::string getGraph() const;

    /**
     * @brief Returns degree (or out-degree in directed graphs).
     * @param v The vertex to get the degree of
     * @return The degree of the vertex
     * @throws std::out_of_range if v is an invalid vertex index
     */
    int degree(int v) const;

    /**
     * @brief Returns in-degree (0 if undirected).
     * @param v The vertex to get the in-degree of
     * @return The in-degree of the vertex
     * @throws std::out_of_range if v is an invalid vertex index
     */
    int inDegree(int v) const;

    /**
     * @brief Checks if the graph is connected.
     * For directed graphs, checks weak connectivity.
     * @return true if the graph is connected, false otherwise
     */
    bool isConnected() const;

    /**
     * @brief Finds an Eulerian circuit in the graph using Hierholzer algorithm.
     * @return A vector containing the vertices in the Eulerian circuit, 
     *         or an empty vector if no Eulerian circuit exists
     */
    std::vector<int> findEulerianCircuit() const;

    
    /**
     * @brief DFS utility function.
     * @param start The starting vertex for DFS
     * @param visited Vector to mark visited vertices
     */
    void dfs(int start, std::vector<bool>& visited) const;
};
}