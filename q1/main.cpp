#include <iostream>
#include "Graph.hpp"

int main() {
    
    // Undirected graph 
    Graph::Graph g(5); // 5 vertices, undirected by default
    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 3);
    g.addEdge(1, 3, 4);
    g.addEdge(3, 4, 5);

    std::cout << "Undirected graph (5 vertices):\n";
    g.printGraph();
    std::cout << "Edge 0-1 exists? " << (g.hasEdge(0,1) ? "yes" : "no")
                << ", weight=" << g.getEdgeWeight(0,1) << "\n";

    // Directed graph 
    Graph::Graph d(4, true); // 4 vertices, directed
    d.addEdge(0, 1, 10);
    d.addEdge(1, 2, 20);
    d.addEdge(2, 3, 30);

    std::cout << "\nDirected graph (4 vertices):\n";
    d.printGraph();
    std::cout << "Edge 1->0 exists? " << (d.hasEdge(1,0) ? "yes" : "no") << "\n";

    // Error handling examples
    try {
        g.addEdge(0, 0); // self-loop, should throw
    } catch (const std::exception &e) {
        std::cout << "\nAttempting self-loop: " << e.what() << "\n";
    }
    try {
        g.addEdge(0, 1); // duplicate, should throw
    } catch (const std::exception &e) {
        std::cout << "Attempting duplicate edge: " << e.what() << "\n";
    }

    return 0;
}
