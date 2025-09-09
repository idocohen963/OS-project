#include "Graph.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <stack>
#include <string>
#include <sstream>

namespace Graph {

/**
 * @brief Constructs a graph with a given number of vertices.
 * @param V Number of vertices
 * @param isDirected Whether the graph is directed (default: false)
 * @throws std::invalid_argument if V is negative or zero
 */
Graph::Graph(int V, bool isDirected) : directed(isDirected), numVertices(V), adj(V) {
    // Ensure the number of vertices is valid
    if (V <= 0) {
        throw std::invalid_argument("Number of vertices must be positive");
    }
    
    // The graph is represented using an adjacency list
    // Each entry adj[v] is a list of pairs (u, w) where:
    //   - u is a vertex adjacent to v
    //   - w is the weight of the edge from v to u
    // For undirected graphs, if (u, w) is in adj[v], then (v, w) is in adj[u]
    // For directed graphs, (u, w) in adj[v] means an edge from v to u with weight w
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

    // Add new edge: Store the destination vertex and the edge weight in the adjacency list
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

    // Search for the edge in the adjacency list and return its weight
    for (size_t i = 0; i < adj[src].size(); i++) {
        if (adj[src][i].first == dest) {
            return adj[src][i].second;  // Return the weight of the edge
        }
    }
    
    return -1;
}

/**
 * @brief Prints the graph to stdout in adjacency-list format.
 * For undirected graphs, both directions are present in the adjacency lists.
 * Format: v -> (neighbor1, w=weight1), (neighbor2, w=weight2), ...
 */
void Graph::printGraph() const {
   
    for (int v = 0; v < numVertices; ++v) {
        std::cout << v << " -> ";  // Print vertex number
        const auto &neighbors = adj[v];  // Get adjacency list for this vertex
        
        // Print all neighbors with their weights
        for (size_t i = 0; i < neighbors.size(); ++i) {
            std::cout << "(" << neighbors[i].first << ", w=" << neighbors[i].second << ")";
            if (i + 1 < neighbors.size()) std::cout << ", ";
        }
        std::cout << std::endl;  
    }
}

/**
 * @brief Prints the graph to stdout in adjacency-list format.
 * For undirected graphs, both directions are present in the adjacency lists.
 * Format: v -> (neighbor1, w=weight1), (neighbor2, w=weight2), ...
 */
std::string Graph::getGraph() const {
   
    std::ostringstream out;
    for (int v = 0; v < numVertices; ++v) {
        out << v << " -> ";  // Print vertex number
        const auto &neighbors = adj[v];  // Get adjacency list for this vertex
        
        // Print all neighbors with their weights
        for (size_t i = 0; i < neighbors.size(); ++i) {
            out << "(" << neighbors[i].first << ", w=" << neighbors[i].second << ")";
            if (i + 1 < neighbors.size()) out << ", ";
        }
        out << std::endl;  
    }
    return out.str();
}

/**
 * @brief Returns degree (or out-degree in directed graphs).
 * @param v Vertex index
 * @return Number of edges connected to vertex v (out-degree for directed graphs)
 * @throws std::out_of_range if v is an invalid vertex index
 */
int Graph::degree(int v) const {
    if (v < 0 || v >= numVertices) throw std::out_of_range("Invalid vertex");
    return static_cast<int>(adj[v].size());
}

/**
 * @brief Returns in-degree (same as degree for undirected graphs).
 * @param v Vertex index
 * @return Number of incoming edges to vertex v (same as degree for undirected graphs)
 * @throws std::out_of_range if v is an invalid vertex index
 */
int Graph::inDegree(int v) const {
    if (v < 0 || v >= numVertices) throw std::out_of_range("Invalid vertex");
    // For undirected graphs, in-degree equals out-degree (both equal to the degree)
    if (!directed) return degree(v);

    // For directed graphs, we need to count all edges pointing to v
    int count = 0;
    for (int u = 0; u < numVertices; ++u) {
        for (auto &p : adj[u]) {
            if (p.first == v) count++;
        }
    }
    return count;
}

/**
 * @brief Performs a depth-first search traversal from a starting vertex.
 * @param start Starting vertex for the DFS traversal
 * @param visited Vector of boolean flags to track visited vertices
 * @throws std::out_of_range if start is an invalid vertex index (through calling functions)
 */
void Graph::dfs(int start, std::vector<bool>& visited) const {
    std::stack<int> st;
    st.push(start);
    visited[start] = true;  // Mark start vertex as visited

    while (!st.empty()) {
        // Pop a vertex from the stack
        int v = st.top();
        st.pop();
        
        // Visit all adjacent vertices that haven't been visited yet
        for (auto &p : adj[v]) {
            int u = p.first;  // Adjacent vertex
            if (!visited[u]) {
                visited[u] = true;  // Mark as visited
                st.push(u);  // Add to stack for further exploration
            }
        }
    }
}

/**
 * @brief Checks if the graph is connected.
 * @details For undirected graphs, checks if all non-isolated vertices are connected.
 *          For directed graphs, checks for weak connectivity (ignoring edge directions).
 * @return true if the graph is connected, false otherwise
 */
bool Graph::isConnected() const {
    // We need a non-isolated vertex as a starting point
    int start = -1;
    for (int i = 0; i < numVertices; ++i) {
        if (!adj[i].empty()) {
            start = i;
            break;
        }
    }
    
    // If no vertex has edges, the graph is considered connected (empty graph)
    if (start == -1) return true; 

    std::vector<bool> visited(numVertices, false);

    if (directed) {
        // For directed graphs, we check weak connectivity
        // A directed graph is weakly connected if replacing all directed edges with
        // undirected edges produces a connected undirected graph
        std::stack<int> st;
        st.push(start);
        visited[start] = true;
        while (!st.empty()) {
            int v = st.top(); st.pop();
            
            // Follow regular edges
            for (auto &p : adj[v]) {
                int u = p.first;
                if (!visited[u]) { visited[u] = true; st.push(u); }
            }
            
            // Also check reverse edges (treating the graph as undirected)
            for (int u = 0; u < numVertices; ++u) {
                for (auto &q : adj[u]) {
                    if (q.first == v && !visited[u]) {
                        visited[u] = true;
                        st.push(u);
                    }
                }
            }
        }
    } else {
        // For undirected graphs, simple DFS is sufficient to check connectivity
        dfs(start, visited);
    }

    // The graph is connected if all non-isolated vertices are reachable
    for (int i = 0; i < numVertices; ++i) {
        if (!adj[i].empty() && !visited[i]) return false;
    }
    return true;
}

/**
 * @brief Finds an Eulerian circuit in the graph using Hierholzer's algorithm.
 * @details For directed graphs, checks if in-degree equals out-degree for all vertices.
 *          For undirected graphs, checks if all vertices have even degree.
 *          Also verifies that the graph is connected.
 * @return A vector containing the vertices in the Eulerian circuit order,
 *         or an empty vector if no Eulerian circuit exists
 */
std::vector<int> Graph::findEulerianCircuit() const {
    
    // 1. The graph must be connected (or weakly connected for directed graphs)
    if (!isConnected()) return {};

    // 2. Check degree conditions:
    //    - For directed graphs: in-degree must equal out-degree for all vertices
    //      (balanced vertices condition)
    //    - For undirected graphs: all vertices must have even degree
    if (directed) {
        for (int v = 0; v < numVertices; ++v) {
            if (inDegree(v) != degree(v)) return {};
        }
    } else {
        for (int v = 0; v < numVertices; ++v) {
            if (degree(v) % 2 != 0) return {};
        }
    }

    // Hierholzer's algorithm implementation:
    // This algorithm finds an Eulerian circuit by building a path and then
    // integrating cycles found along the way
    std::vector<std::vector<std::pair<int,int>>> tempAdj = adj; // copy to consume edges
    std::vector<int> circuit;
    std::stack<int> st;

    // Find a starting vertex with edges
    int start = 0;
    while (start < numVertices && tempAdj[start].empty()) start++;
    if (start == numVertices) return {}; // no edges at all

    // Start the traversal
    st.push(start);
    while (!st.empty()) {
        int v = st.top();
        if (!tempAdj[v].empty()) {
            // If vertex has unexplored edges, take one
            int u = tempAdj[v].back().first;
            tempAdj[v].pop_back(); // Remove the edge from v to u
            
            if (!directed) {
                // For undirected graphs, we also need to remove the corresponding reverse edge
                // from u to v to avoid traversing the same edge twice
                auto &neigh = tempAdj[u];
                for (auto it = neigh.begin(); it != neigh.end(); ++it) {
                    if (it->first == v) { neigh.erase(it); break; }
                }
            }
            // Move to the next vertex
            st.push(u);
        } else {
            // If no more unexplored edges, add vertex to circuit and backtrack
            circuit.push_back(v);
            st.pop();
        }
    }

    // Reverse the circuit to get the correct order
    std::reverse(circuit.begin(), circuit.end());
    return circuit;
}

}

