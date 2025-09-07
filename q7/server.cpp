// ==========================
// File: euler_server.cpp
// TCP server that receives a graph (or parameters for a random graph),
// runs the Euler cycle algorithm, and returns the result to the client.
// ==========================

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <random>
#include <algorithm>

#include "Graph.hpp"
#include "algorithms/AlgorithmFactory.hpp"

// ==========================
// Request Parsing
// ==========================

struct ParsedRequest
{
    enum Algorithm
    {
        ALG_MST = 1,
        ALG_SCC = 2,
        ALG_MaxFlow = 3,
        ALG_CliqueCount = 4

    } alg;

    enum Kind
    {
        RANDOM,
        MANUAL
    } kind;

    int v{};                                // vertices
    int e{};                                // edges
    int s{};                                // seed (FOR RANDOM)
    bool d{};                               // for directed or undirected graph
    std::vector<std::pair<int, int>> edges; // for MANUAL
};

static bool read_required_alg(std::istringstream &in, int &alg_out, std::string &err)
{
    std::string key;
    if (!(in >> key))
    {
        err = "Missing ALG token";
        return false;
    }
    if (key != "ALG")
    {
        err = "Expected 'ALG' token";
        return false;
    }
    if (!(in >> alg_out))
    {
        err = "Missing algorithm id after ALG";
        return false;
    }
    if (alg_out < 1 || alg_out > 4)
    {
        err = "Invalid algorithm id (valid:ALG_MST = 1,ALG_SCC = 2, ALG_MaxFlow = 3, ALG_CliqueCount = 4)";
        return false;
    }
    return true;
}

static std::string trim(const std::string &s)
{
    size_t a = s.find_first_not_of(" \r\n\t");
    if (a == std::string::npos)
        return "";
    size_t b = s.find_last_not_of(" \r\n\t");
    return s.substr(a, b - a + 1);
}

static std::optional<ParsedRequest> parse_request(const std::string &text, std::string &err)
{
    std::istringstream in(text);
    std::string tag;
    if (!(in >> tag))
    {
        err = "Empty request";
        return std::nullopt;
    }

    if (tag == "RANDOM")
    {
        ParsedRequest r;
        r.kind = ParsedRequest::RANDOM;

        if (!(in >> r.v >> r.e >> r.s))
        {
            err = "Usage: RANDOM <vertices> <edges> <seed> [directed] ALG <id> \n";
            return std::nullopt;
        }

        int dirFlag;
        if (in >> dirFlag)
        {
            r.d = (dirFlag != 0);
        }

        int alg_id = 0;
        if (!read_required_alg(in, alg_id, err))
            return std::nullopt;
        r.alg = static_cast<ParsedRequest::Algorithm>(alg_id);

        if (r.v <= 0 || r.e < 0)
        {
            err = " parameters not actual or positive numbers.";
            return std::nullopt;
        }
        return r;
        err = "Unknown request";
        return std::nullopt;
    }

    else if (tag == "MANUAL")
    {
        ParsedRequest r;
        r.kind = ParsedRequest::MANUAL;
        if (!(in >> r.v >> r.e))
        {
            err = "Usage: MANUAL <vertices> <edges> [directed]\n"
                  "<edge_1_src> <edge_1_dest> [weight_1]  ... <edge_N_src> <edge_N_dest> [weight_N]\n";
            return std::nullopt;
        }

        int dirFlag;
        if (in >> dirFlag)
        {
            r.d = (dirFlag != 0);
        }

        int alg_id = 0;
        if (!read_required_alg(in, alg_id, err))
            return std::nullopt;
        r.alg = static_cast<ParsedRequest::Algorithm>(alg_id);

        if (r.v <= 0 || r.e < 0)
        {
            err = "Invalid vertices or edges\n";
            return std::nullopt;
        }

        r.edges.reserve((size_t)r.e);
        for (int i = 0; i < r.e; ++i)
        {
            int src, dest;
            if (!(in >> src >> dest))
            {
                err = "Not enough edges provided\n";
                return std::nullopt;
            }
            if (src < 0 || src >= r.v || dest < 0 || dest >= r.v)
            {
                err = "Edge vertices out of range\n";
                return std::nullopt;
            }
            r.edges.emplace_back(src, dest);
        }
        return r;
    }
    // else
    err = "Unknown request type\n";
    return std::nullopt;
}

static Graph::Graph build_graph_from_edges(int n, const std::vector<std::pair<int, int>> &edges, bool directed = false)
{
    Graph::Graph g(n, directed);
    for (auto [u, v] : edges)
        g.addEdge(u, v);
    return g;
}

static Graph::Graph build_random_graph(int vertices, int edges, int seed, bool directed)
{
    if (vertices <= 0 || edges < 0)
        throw std::invalid_argument("Invalid vertices or edges");

    int maxEdges = directed
                       ? vertices * (vertices - 1)      // directed graph
                       : vertices * (vertices - 1) / 2; // undirected graph

    if (edges > maxEdges)
        throw std::invalid_argument("Too many edges for the given number of vertices");

    // Create a graph with x vertices as input
    Graph::Graph g(vertices, directed);

    // set random values:
    //  Initialize random number generator with the user-provided seed
    std::mt19937 rng(seed);
    // Create a uniform distribution over [0, vertices-1] for selecting random vertices
    std::uniform_int_distribution<int> dist(0, vertices - 1);

    // add edges within the uniform range
    int added = 0;
    while (added < edges)
    {
        int u = dist(rng);
        int v = dist(rng);
        if (u != v && !g.hasEdge(u, v))
        {
            g.addEdge(u, v);
            ++added;
        }
    }

    return g;
}

// ==========================
// Networking helpers
// ==========================

static ssize_t recv_all(int fd, std::string &out)
{
    char buf[4096];
    ssize_t total = 0;
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0)
    {
        out.append(buf, buf + n);
        total += n;
        if (out.size() >= 2 && (out.rfind("\n\n") == out.size() - 2 || out.rfind("\r\n\r\n") == out.size() - 4))
            break;
    }
    return (n < 0 && errno != 0) ? -1 : total;
}

static bool send_all(int fd, const std::string &s)
{
    size_t sent = 0;
    const char *p = s.c_str();
    size_t left = s.size();
    while (left > 0)
    {
        ssize_t n = ::send(fd, p + sent, left, 0);
        if (n <= 0)
            return false;
        sent += (size_t)n;
        left -= (size_t)n;
    }
    return true;
}

//===========================
// algorithm handling
//===========================

/*std::string run_euler(Graph::Graph g)
{
    auto circuit = g.findEulerianCircuit();
    std::ostringstream out;

    if (!circuit.empty())
    {
        out << "Eulerian Circuit: ";
        for (size_t i = 0; i < circuit.size(); ++i)
        {
            out << circuit[i];
            if (i + 1 < circuit.size())
                out << " -> ";
        }
        out << "\n";
    }
    else
    {
        out << "No Eulerian circuit exists in this graph.\n";
    }

    return out.str();
}

std::string run_scc(Graph::Graph g)
{
    // Placeholder for SCC algorithm implementation
    std::ostringstream out;
    out << "SCC algorithm: not implemented yet.\n";
    return out.str();
}
*/

// ==========================
// Handle a client connection
// ==========================

static void handle_client(int cfd)
{
    std::string req_raw;
    if (recv_all(cfd, req_raw) < 0)
    {
        send_all(cfd, "ERROR: recv failed\n");
        return;
    }
    std::string err;
    auto parsed = parse_request(trim(req_raw), err);
    if (!parsed)
    {
        send_all(cfd, std::string("ERROR: ") + err + "\n");
        return;
    }

    try
    {
        // Build the graph
        Graph::Graph g = (parsed->kind == ParsedRequest::RANDOM)
                             ? build_random_graph(parsed->v, parsed->e, parsed->s, parsed->d)
                             : build_graph_from_edges(parsed->v, parsed->edges, parsed->d);

        // Send back the graph representation
        std::ostringstream out;
        out << g.getGraph();
        send_all(cfd, out.str());

        // Check if algorithm requires directed graph
        if ((parsed->alg == ParsedRequest::ALG_SCC || parsed->alg == ParsedRequest::ALG_MaxFlow) && !g.isDirected())
        {
            std::string err = "Error: This algorithm requires a directed graph.\n";
            send_all(cfd, err);
            return;
        }

        // Run the requested algorithm
        std::string result;
        Algorithm* alg = AlgorithmFactory::create(parsed->alg);
        if (alg) {
            result = alg->run(g);
            delete alg;
        } else {
            result = "ERROR: invalid algorithm id\n";
        }
        send_all(cfd, result);
    }

    catch (const std::exception &ex)
    {
        send_all(cfd, std::string("ERROR: ") + ex.what() + "\n");
    }
}

// ==========================
// Main
// ==========================

static volatile sig_atomic_t g_stop = 0;
static int sfd = -1;
static void on_sigint(int)
{
    g_stop = 1;
    if (sfd != -1)
    {
        ::close(sfd);
        sfd = -1;
    }
}

int main(int argc, char **argv)
{
    int port = 8080;
    int opt;
    while ((opt = ::getopt(argc, argv, "p:")) != -1)
    {
        switch (opt)
        {
        case 'p':
            port = std::atoi(optarg);
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " -p <port>\n";
            return 1;
        }
    }

    std::signal(SIGINT, on_sigint);

    sfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0)
    {
        perror("socket");
        return 1;
    }

    int yes = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(sfd, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return 1;
    }
    if (::listen(sfd, 16) < 0)
    {
        perror("listen");
        return 1;
    }

    std::cout << "[EulerServer] Listening on port " << port << "... (Ctrl+C to stop)\n";
    while (!g_stop)
    {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = ::accept(sfd, (sockaddr *)&caddr, &clen);
        if (cfd < 0)
        {
            if (errno == EINTR && g_stop)
                break;
            perror("accept");
            continue;
        }
        handle_client(cfd);
        ::close(cfd);
    }
    ::close(sfd);
    std::cout << "[EulerServer] Stopped.\n";
    return 0;
}
