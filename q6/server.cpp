// ==========================
// File: server.cpp
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

// ==========================
// Request Parsing
// ==========================

struct ParsedRequest
{
    int v{};         // vertices
    int e{};         // edges
    int s{};         // for seed
    bool directed{}; // for directed or undirected graph
};

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
    ParsedRequest r;
    if (!(in >> r.v >> r.e >> r.s))
    {
        err = "Usage: -v vertices -e edges -s seed [-d] \n";
        return std::nullopt;
    }
    
    int dirFlag;
    if (in >> dirFlag)
    {
        r.directed = (dirFlag != 0);
    }

    if (r.v <= 0 || r.e < 0)
    {
        err = " parameters not actual or positive numbers.";
        return std::nullopt;
    }
    return r;
    err = "Unknown request";
    return std::nullopt;
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
        Graph::Graph g = build_random_graph(parsed->v, parsed->e, parsed->s, parsed->directed);

        std::ostringstream out;
        out << g.getGraph();
        send_all(cfd, out.str());

        // Find and send Eulerian circuit if exists
        auto circuit = g.findEulerianCircuit();
        if (!circuit.empty())
        {
            std::ostringstream out;
            out << "Eulerian Circuit: ";
            for (size_t i = 0; i < circuit.size(); ++i)
            {
                out << circuit[i];
                if (i + 1 < circuit.size())
                    out << " -> ";
            }
            out << "\n";
            send_all(cfd, out.str());
        }
        else
        {
            send_all(cfd, "No Eulerian circuit exists in this graph.\n");
        }
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
