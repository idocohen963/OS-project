#ifndef SCC_HPP
#define SCC_HPP

#include "Graph.hpp"
#include <vector>
#include <string>

class SCC {
public:
    std::string run(const Graph::Graph& graph);
private:
    void dfs(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& order) const;
    void dfsRev(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& component) const;
};

#endif // SCC_HPP