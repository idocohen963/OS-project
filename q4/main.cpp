#include <iostream>
#include "Graph.hpp"
#include <getopt.h>
#include <stdexcept>
#include <random>

// Eulerian circuit Example: ./demo -v 4 -e 4 -s 4

int main(int argc, char* argv[])  {
    int vertices = -1, edges = -1, seed = -1;
    int opt;

    while ((opt = getopt(argc, argv, "v:e:s:")) != -1) {
        switch (opt) {
            case 'v': vertices = std::stoi(optarg); break;
            case 'e': edges = std::stoi(optarg); break;
            case 's': seed = std::stoi(optarg); break;
            default:
                std::cerr << "Usage: " << argv[0] << " -v vertices -e edges -s seed\n";
                return 1;
        }
    }

    // Check that all required parameters were provided 
    if (vertices == -1 || edges == -1 ||  seed == -1) {
        std::cerr << "Usage: " << argv[0] << " -v vertices -e edges -s seed\n";
        return 1;
    }

    // If values are invalid
    if (vertices <= 0 || edges < 0) {
        throw std::invalid_argument(" parameters not actual or positive numbers.");
    }
    
    
    // Create a graph with x vertices as input
    Graph::Graph g(vertices);

    //set random values:
    // Initialize random number generator with the user-provided seed
    std::mt19937 rng(seed);
    // Create a uniform distribution over [0, vertices-1] for selecting random vertices
    std::uniform_int_distribution<int> dist(0, vertices - 1);
    
    //add edges within the uniform range
    int added = 0;
    while (added < edges) {
        int u = dist(rng);
        int v = dist(rng);
        if (u != v && !g.hasEdge(u, v)) {
            g.addEdge(u, v);
            ++added;
        }
    }

    // Print the graph
    g.printGraph();

    // Find and print Eulerian circuit if exists
    std::vector<int> circuit = g.findEulerianCircuit();
    if (!circuit.empty()) {
        std::cout << "Eulerian Circuit: ";
        for (size_t i = 0; i < circuit.size(); ++i) {
            std::cout << circuit[i];
            if (i + 1 < circuit.size()) std::cout << " -> ";
        }
        std::cout << std::endl;
    }else {
         std::cout << std::endl << "No Eulerian circuit exists in this graph.\n";
    }
    
    return 0;

}