#include <iostream>
#include "Graph.hpp"

int main() {
    
    std::cout << "\n=== Euler Circuit Demonstration ===\n";
    
    // Creating an undirected graph with 4 vertices that has an Eulerian circuit
    // All vertices need to have even degrees for an Eulerian circuit to exist
    Graph::Graph eulerGraph(4);
    
    // Creating a basic circuit: 0-1-2-3-0
    eulerGraph.addEdge(0, 1);
    eulerGraph.addEdge(1, 2);
    eulerGraph.addEdge(2, 3);
    eulerGraph.addEdge(3, 0);
    
    std::cout << "Graph with an Eulerian circuit:\n";
    eulerGraph.printGraph();
    
    // Finding Eulerian circuit
    std::vector<int> circuit = eulerGraph.findEulerianCircuit();
    
    std::cout << "\nEulerian circuit found: ";
    for (size_t i = 0; i < circuit.size(); ++i) {
        std::cout << circuit[i];
        if (i < circuit.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
 
    // Example of a graph without an Eulerian circuit
    std::cout << "\n=== Graph Without an Eulerian Circuit ===\n";
    Graph::Graph noEulerGraph(4);
    noEulerGraph.addEdge(0, 1);
    noEulerGraph.addEdge(1, 2);
    noEulerGraph.addEdge(2, 3);
    // Missing edge 3-0 to complete the circuit, thus some vertices have odd degrees
    
    std::cout << "Graph without an Eulerian circuit:\n";
    noEulerGraph.printGraph();
    
    std::vector<int> noCircuit = noEulerGraph.findEulerianCircuit();
    if (noCircuit.empty()) {
        std::cout << std::endl << "No Eulerian circuit exists in this graph.\n";
    }    
    
    return 0;
}
