#include "SCC.hpp"
#include <string>
#include <vector>

void SCC::dfs(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& order) const {
    visited[v] = true;
    for (int u = 0; u < g.getNumVertices(); ++u) {
        if (g.hasEdge(v, u) && !visited[u]) dfs(g, u, visited, order);
    }
    order.push_back(v);
}

void SCC::dfsRev(const Graph::Graph& g, int v, std::vector<bool>& visited, std::vector<int>& component) const {
    visited[v] = true;
    component.push_back(v);
    for (int u = 0; u < g.getNumVertices(); ++u) {
        if (g.hasEdge(u, v) && !visited[u]) dfsRev(g, u, visited, component);
    }
}

std::string SCC::run(const Graph::Graph& graph) {
    int n = graph.getNumVertices();
    std::vector<bool> visited(n, false);
    std::vector<int> order;
    for (int i = 0; i < n; ++i)
        if (!visited[i]) dfs(graph, i, visited, order);
    std::fill(visited.begin(), visited.end(), false);
    int sccCount = 0;
    std::vector<std::vector<int>> sccs;
    for (int i = n - 1; i >= 0; --i) {
        int v = order[i];
        if (!visited[v]) {
            std::vector<int> comp;
            dfsRev(graph, v, visited, comp);
            sccs.push_back(comp);
            ++sccCount;
        }
    }
    std::string result = "Strongly connected components:";
    for (size_t i = 0; i < sccs.size(); ++i) {
        result += "\nComponent " + std::to_string(i+1) + ": ";
        for (size_t j = 0; j < sccs[i].size(); ++j) {
            result += std::to_string(sccs[i][j]);
            if (j+1 < sccs[i].size()) result += ", ";
        }
    }
    return result;
}