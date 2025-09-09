/**
 * @file server.cpp
 * @brief Multithreaded TCP server implementing Leader/Follower pattern for running graph algorithms.
 *
 * The server accepts two request types (RANDOM and MANUAL). For RANDOM the client
 * provides (vertices, edges, seed, directed_flag). For MANUAL the client supplies
 * an explicit list of edges. The server builds the graph, runs all available
 * algorithms (via AlgorithmFactory) and returns a textual report.
 *
 * Uses a strict Leader/Follower concurrency pattern where only one thread (the Leader)
 * blocks on accept() at any given time, while all other threads (Followers) wait on
 * a condition variable. When the Leader accepts a connection, it immediately promotes
 * a Follower to become the new Leader and then handles the client as a Worker.
 */

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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#include "Graph.hpp"
#include "algorithms/AlgorithmFactory.hpp"

// ==========================
// Request Parsing
// ==========================

/**
 * @struct ParsedRequest
 * @brief In-memory representation of a parsed client request.
 *
 * Fields:
 * - kind: request type (RANDOM or MANUAL)
 * - v: number of vertices
 * - e: number of edges
 * - s: RNG seed (only valid for RANDOM)
 * - d: directed flag
 * - edges: list of (src,dest) pairs when kind==MANUAL
 */
struct ParsedRequest
{
    enum Kind
    {
        RANDOM,
        MANUAL
    } kind;

    int v{};                                ///< vertices
    int e{};                                ///< edges
    int s{};                                ///< seed (FOR RANDOM)
    bool d{};                               ///< for directed or undirected graph
    std::vector<std::pair<int, int>> edges; ///< for MANUAL
};

/**
 * @brief Trim leading and trailing whitespace from a string.
 *
 * @param s Input string.
 * @return Trimmed string.
 */
static std::string trim(const std::string &s)
{
    size_t a = s.find_first_not_of(" \r\n\t");
    if (a == std::string::npos)
        return "";
    size_t b = s.find_last_not_of(" \r\n\t");
    return s.substr(a, b - a + 1);
}

/**
 * @brief Parse a textual request received from the client.
 *
 * The supported formats are:
 * - RANDOM <vertices> <edges> <seed> [directed]
 * - MANUAL <vertices> <edges> [directed]\n<edge lines...>
 *
 * On success returns a ParsedRequest; on failure returns std::nullopt and sets `err`.
 *
 * @param text Raw input text from the client (trimmed prior to call is acceptable).
 * @param err Output parameter populated with an error message on failure.
 * @return std::optional<ParsedRequest>
 */
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
            err = "Usage: RANDOM <vertices> <edges> <seed> [directed]\n";
            return std::nullopt;
        }

        int dirFlag;
        if (in >> dirFlag)
        {
            r.d = (dirFlag != 0);
        }

        if (r.v <= 0 || r.e < 0)
        {
            err = " parameters not actual or positive numbers.";
            return std::nullopt;
        }
        return r;
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

/**
 * @brief Construct a Graph object from an explicit list of edges.
 *
 * @param n Number of vertices.
 * @param edges Vector of (src,dest) pairs.
 * @param directed Whether the graph is directed.
 * @return Graph::Graph instance owning the constructed adjacency lists.
 */
static Graph::Graph build_graph_from_edges(int n, const std::vector<std::pair<int, int>> &edges, bool directed = false)
{
    Graph::Graph g(n, directed);
    for (auto [u, v] : edges)
        g.addEdge(u, v);
    return g;
}

/**
 * @brief Build a random graph with the specified parameters.
 *
 * Uses std::mt19937 seeded with `seed` and uniformly selects vertex pairs.
 * Throws std::invalid_argument on invalid parameters.
 *
 * @param vertices Number of vertices.
 * @param edges Number of edges to add (duplicates/self-loops avoided).
 * @param seed RNG seed.
 * @param directed Whether the graph is directed.
 * @return Graph::Graph constructed graph.
 */
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

/**
 * @brief Receive data from fd until the client shuts down the write side or a terminator is seen.
 *
 * The function appends received bytes to `out` and stops when it detects a double newline
 * terminator ("\n\n" or "\r\n\r\n") or when recv returns 0. On socket error returns -1.
 *
 * @param fd Socket file descriptor.
 * @param out Output string to append received bytes to.
 * @return total number of bytes received, or -1 on error.
 */
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

/**
 * @brief Send the full contents of a string over a blocking socket.
 *
 * Retries until all bytes are sent or an error occurs.
 *
 * @param fd Socket file descriptor.
 * @param s Data to send.
 * @return true on success, false on error.
 */
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
// Handle a client connection - runs all 4 algorithms
// ==========================

/**
 * @brief Handle a single client connection.
 *
 * This function receives a request, parses it, constructs the graph (random or manual),
 * runs all algorithms via AlgorithmFactory and sends back a textual report. Errors are
 * reported back to the client as a short message starting with "ERROR:".
 *
 * @param cfd Connected client socket file descriptor. The caller retains responsibility
 *            for closing the descriptor after this function returns.
 */
static void handle_client(int cfd)
{
    std::string req_raw;
    // Receive the full request from the client. recv_all will return -1 on socket error.
    // The protocol uses a blank-line terminator ("\n\n" or CRLF pair) or EOF to finish the request.
    if (recv_all(cfd, req_raw) < 0)
    {
        send_all(cfd, "ERROR: recv failed\n");
        return;
    }
    std::string err;
    // Trim and parse the request. parse_request validates parameter counts and ranges.
    auto parsed = parse_request(trim(req_raw), err);
    if (!parsed)
    {
        // Send a short error back to the client and return.
        send_all(cfd, std::string("ERROR: ") + err + "\n");
        return;
    }

    try
    {
    // Build the graph (either random or from explicit edges)
        Graph::Graph g = (parsed->kind == ParsedRequest::RANDOM)
                             ? build_random_graph(parsed->v, parsed->e, parsed->s, parsed->d)
                             : build_graph_from_edges(parsed->v, parsed->edges, parsed->d);

        // Send back the graph representation
        std::ostringstream out;
        out << "Graph:\n" << g.getGraph() << "\nResults:\n";

        // Run each algorithm and append its textual result.
        for (int alg_id = 1; alg_id <= 4; ++alg_id) {
            try {
                // Algorithms 2 and 3 require a directed graph; if the input is undirected,
                // report that requirement instead of running them.
                if ((alg_id == 2 || alg_id == 3) && !g.isDirected()) {
                    out << "Algorithm " << alg_id << ": Requires directed graph\n\n";
                    continue;
                }

                // Create, run and free the algorithm instance. The run(...) call is
                // expected to return a string describing the result; any thrown exceptions
                // are caught below to avoid crashing the connection handler.
                Algorithm* alg = AlgorithmFactory::create(alg_id);
                if (alg) {
                    out  << alg->run(g) << "\n\n";
                    delete alg;
                }
            } catch (const std::exception &ex) {
                // Convert algorithm failure into a textual message sent back to the client.
                out << "Algorithm " << alg_id << " Error: " << ex.what() << "\n\n";
            }
        }

        send_all(cfd, out.str());
    }
    catch (const std::exception &ex)
    {
        send_all(cfd, std::string("ERROR: ") + ex.what() + "\n");
    }
}

/**
 * Leader/Follower pattern globals
 * A mutex, condition variable, and leader token flag implement the Leader/Follower
 * synchronization mechanism. Only one thread (the Leader) blocks on accept() at a time,
 * while all other threads (Followers) wait on the condition variable.
 */

std::mutex leader_mutex;                   // mutex protecting the leader token
std::condition_variable leader_condition;  // follower wakeup condition variable
bool leader_token_available = true;       // flag indicating if leader token is available
std::atomic<bool> server_stop(false);     // flag to request worker shutdown

/**
 * @brief Worker thread main loop implementing Leader/Follower pattern.
 *
 * Each worker thread competes for the Leader role. Only the Leader thread
 * blocks on accept() while all other threads (Followers) wait on the condition
 * variable. When the Leader accepts a connection, it promotes a Follower to
 * become the new Leader and then handles the client connection.
 *
 * @param sfd Listening socket file descriptor passed from main.
 */
void worker_thread(int sfd) {
    while (!server_stop) {
        int client_fd = -1;
        
        // Phase 1: Compete for Leader role
        {
            std::unique_lock<std::mutex> lock(leader_mutex);
            // Wait until leader token becomes available or server should stop
            leader_condition.wait(lock, []{ return leader_token_available || server_stop; });
            
            // If server should stop, exit the loop
            if (server_stop) break;
            
            // Acquire the leader token - become the Leader
            leader_token_available = false;
        }
        
        // Phase 2: Leader accepts new connection
        if (!server_stop) {
            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);
            client_fd = ::accept(sfd, (sockaddr *)&caddr, &clen);
            
            // Phase 3: Promote a new Leader immediately after accepting
            {
                std::lock_guard<std::mutex> lock(leader_mutex);
                leader_token_available = true;
            }
            leader_condition.notify_one(); // Wake one Follower to become new Leader
            
            // Phase 4: Handle the client connection (now as a Worker)
            if (client_fd >= 0) {
                handle_client(client_fd);
                ::close(client_fd);
            } else {
                // accept() failed (likely due to server shutdown)
                if (server_stop) {
                    break;
                }
                // On accept error, continue to next iteration
            }
        }
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
    // Request workers to stop and wake all waiting Followers
    server_stop = true;
    std::cout << "recive Ctrl+C from the user.\n";
    /*
    leader_condition.notify_all(); // wake all worker threads so they can exit
    if (sfd != -1)
    {
        ::close(sfd);
        sfd = -1;
    }*/
}

/**
 * @brief Server entry point.
 *
 * Command-line options:
 * -p <port>    Listening port (default 8080)
 * -t <threads> Number of worker threads (default 4)
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return exit status.
 */
int main(int argc, char **argv)
{
    int port = 8080;
    int num_threads = 10; // default number of worker threads
    int opt;

    // Allow overriding the number of worker threads via a command-line option
    while ((opt = ::getopt(argc, argv, "p:t:")) != -1)
    {
        switch (opt)
        {
        case 'p':
            port = std::atoi(optarg);
            break;
        case 't':
            num_threads = std::atoi(optarg);
            if (num_threads <= 0) {
                std::cerr << "Number of threads must be positive\n";
                return 1;
            }
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " -p <port> -t <threads>\n";
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

    // Create worker threads implementing Leader/Follower pattern
    // Pass the listening socket file descriptor to each worker thread
    std::vector<std::thread> workers;
    for (int i = 0; i < num_threads; ++i) {
        workers.emplace_back(worker_thread, sfd);
    }

    std::cout << "[MultiThreadServer] Listening on port " << port 
              << " with " << num_threads << " threads (Leader/Follower pattern)\n\n (Ctrl+C to stop)\n" ;
    
    // Wait for shutdown signal - no accept loop in main with Leader/Follower pattern
    // The worker threads handle all accept() calls through the Leader/Follower mechanism
    while (!g_stop) {
    }
    
    // Shutdown the server and join worker threads
    server_stop = true;
    leader_condition.notify_all();
    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    
    ::close(sfd);
    std::cout << "[MultiThreadServer] Stopped.\n";
    return 0;
}
