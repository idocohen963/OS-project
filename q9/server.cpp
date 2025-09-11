/**
 * @file server.cpp
 * @brief Multithreaded TCP server using a staged Pipeline to run graph algorithms.
 *
 * The server accepts two request types (RANDOM and MANUAL). For RANDOM, the client
 * provides (vertices, edges, seed, directed_flag). For MANUAL, the client supplies
 * an explicit list of edges. The server builds the graph, runs all available
 * algorithms (via AlgorithmFactory), and returns a textual report.
 *
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
#include <memory>

#include "Graph.hpp"
#include "algorithms/AlgorithmFactory.hpp"

// ==========================
// Request Parsing
// ==========================

/**
 * @struct ParsedRequest
 * @brief Represents a parsed client request in memory.
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
 * Removes spaces, tabs, and newline characters from both ends of the input string.
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
 * Supported formats:
 * - RANDOM <vertices> <edges> <seed> [directed]
 * - MANUAL <vertices> <edges> [directed]\n<edge lines...>
 *
 * On success, returns a ParsedRequest. On failure, returns std::nullopt and sets `err`.
 *
 * @param text Raw input text from the client (can be pre-trimmed).
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
 * Creates a graph using the provided vertices and edges, with an option to specify
 * whether the graph is directed.
 *
 * @param n Number of vertices.
 * @param edges Vector of (src, dest) pairs.
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
 * @brief Receive data from a socket until the client shuts down the write side or a terminator is seen.
 *
 * Appends received bytes to `out` and stops when a double newline terminator ("\n\n" or "\r\n\r\n")
 * is detected or when recv returns 0. On socket error, returns -1.
 *
 * @param fd Socket file descriptor.
 * @param out Output string to append received bytes to.
 * @return Total number of bytes received, or -1 on error.
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

// ======================
// Bounded thread-safe queue (backpressure)
// ======================

/**
 * @tparam T payload type passed between adjacent pipeline stages.
 *
 * A minimal bounded MPMC queue implemented with std::mutex + condition_variables.
 * The capacity provides *backpressure*: when a downstream stage is slower,
 * upstream producers will block in push(), preventing unbounded memory growth.
 *
 * Close semantics:
 *  - close(): waking all blocked threads and making future push/pop fail fast.
 *  - pop(): returns false when the queue is closed *and* empty -> stage should exit.
 *  - push(): returns false when the queue has been closed -> producer should drop/cleanup.
 *
 * Note: This is intentionally simple and portable. If you need higher throughput,
 * replace with a lock-free MPMC or a library queue keeping the same interface.
 */
template <typename T>
class BoundedQueue
{
private:
    std::mutex m_;
    std::condition_variable cv_not_full_, cv_not_empty_;
    std::queue<T> q_;
    const size_t cap_;
    bool closed_ = false;

public:
    explicit BoundedQueue(size_t cap) : cap_(cap) {}

    /**
     * @brief Enqueue a value, blocking when the queue is full.
     * @return false if the queue is closed (value not enqueued).
     */
    bool push(T v)
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_not_full_.wait(lk, [&]
                          { return q_.size() < cap_ || closed_; });
        if (closed_)
            return false;
        q_.push(std::move(v));
        cv_not_empty_.notify_one();
        return true;
    }

    /**
     * @brief Dequeue a value, blocking when the queue is empty.
     * @return false if the queue is closed and empty (producer finished)
     */
    bool pop(T &out)
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_not_empty_.wait(lk, [&]
                           { return !q_.empty() || closed_; });
        if (q_.empty())
            return false;
        out = std::move(q_.front());
        q_.pop();
        cv_not_full_.notify_one();
        return true;
    }

    /**
     * @brief Close the queue, waking all blocked threads.
     *
     * After close(), push() and pop() will return false immediately.
     * pop() will return false only when the queue is both closed and empty.
     */
    void close()
    {
        std::lock_guard<std::mutex> lk(m_);
        closed_ = true;
        cv_not_empty_.notify_all();
        cv_not_full_.notify_all();
    }
};

// ===== Globals =====

// Used for graceful shutdown on SIGINT
static std::atomic<bool> g_stop{false};
// Listening socket file descriptor
static int sfd = -1;

// ======================
// Pipeline payload types
// ======================

// Payloads passed between adjacent pipeline stages
struct Conn
{
    int fd;
};

// Received raw request buffer
struct RecvBuf
{
    int fd;
    std::string raw;
};

// Parsed request
struct Req
{
    int fd;
    ParsedRequest pr;
};

// 
/**
 * @brief Shared state for collecting results from multiple algorithm tasks.
 *
 * All RunTask instances for the *same* client share a single ResultCollector.
 * 'remaining' counts how many algorithms are still running; when it reaches 0,
 * the final combined output is enqueued to the send stage.
 */
struct ResultCollector
{
    std::mutex m;
    std::ostringstream out;
    std::atomic<int> remaining{0};
    int fd{-1};
};

/**
 * @brief A single algorithm run for a given graph.
 *
 * Instead of "one task runs all algorithms", we create one RunTask
 * per algorithm id, all sharing the same Graph (via shared_ptr) and ResultCollector.
 */
struct RunTask
{
    std::shared_ptr<ResultCollector> rc;
    std::shared_ptr<const Graph::Graph> g;
    int alg_id;
};

/**
 * @brief Final combined reply for a client, ready to send().
 */
struct Result
{
    int fd;
    std::string out;
};

// ====================
// Pipeline stages 
// ====================

/**
 * @brief Accept new client connections.
 *
 * Blocks in accept() until a client connects. On success, the connection
 * is wrapped in a Conn and pushed to the recv stage. On SIGINT, the
 * listening socket is shut down, unblocking accept() so this loop can exit.
 *
 * @param sfd  Listening socket file descriptor.
 * @param qA2R Queue for accepted connections (Conn).
 */
static void stage_accept(int sfd, BoundedQueue<Conn> &qA2R)
{
    while (!g_stop.load())
    {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int cfd = ::accept(sfd, (sockaddr *)&caddr, &clen);
        if (cfd < 0)
        {
            if (g_stop.load())
                break;
            continue;
        }
        if (!qA2R.push(Conn{cfd}))
        {
            ::close(cfd); // queue closed; refuse new client
            break;
        }
    }
}

/**
 * @brief Receive raw request text from a client socket.
 *
 * Reads until a terminator ("\n\n" or "\r\n\r\n") or EOF. Produces a RecvBuf
 * containing the client fd and the request string. On socket error, the
 * connection is closed. If the downstream queue is closed, the connection is dropped.
 *
 * @param qA2R Input queue of accepted connections.
 * @param qR2P Output queue of raw request buffers.
 */
static void stage_recv(BoundedQueue<Conn> &qA2R, BoundedQueue<RecvBuf> &qR2P)
{
    Conn c;
    while (qA2R.pop(c))
    {
        std::string raw;
        if (recv_all(c.fd, raw) < 0)
        {
            ::close(c.fd);
            continue;
        }
        if (!qR2P.push(RecvBuf{c.fd, std::move(raw)}))
        {
            ::close(c.fd);
        }
    }
}

/**
 * @brief Parse a raw request string into a structured Req.
 *
 * On success, produces a Req object and pushes it to the build stage.
 * On parse error, sends "ERROR: ..." back to the client and closes the socket.
 *
 * @param qR2P Input queue of raw request buffers.
 * @param qP2B Output queue of parsed requests.
 */
static void stage_parse(BoundedQueue<RecvBuf> &qR2P, BoundedQueue<Req> &qP2B)
{
    RecvBuf rb;
    while (qR2P.pop(rb))
    {
        std::string err;
        auto pr = parse_request(trim(rb.raw), err);
        if (!pr)
        {
            send_all(rb.fd, "ERROR: " + err + "\n");
            ::close(rb.fd);
            continue;
        }
        qP2B.push(Req{rb.fd, *pr});
    }
}

/**
 * @brief Build a Graph and fan out RunTasks for each algorithm.
 *
 * For each Req:
 *  - Builds a Graph (random or manual).
 *  - Creates a ResultCollector shared across all algorithms for this client.
 *  - Pushes one RunTask per algorithm id into the run stage.
 *
 * On error, sends "ERROR: ..." back to the client and closes the socket.
 *
 * @param qP2B Input queue of parsed requests.
 * @param qRun Output queue of RunTasks for algorithm execution.
 */
static void stage_build(BoundedQueue<Req> &qP2B, BoundedQueue<RunTask> &qRun)
{
    Req rq;
    while (qP2B.pop(rq))
    {
        try
        {
            auto gptr = std::make_shared<const Graph::Graph>(
                (rq.pr.kind == ParsedRequest::RANDOM)
                    ? build_random_graph(rq.pr.v, rq.pr.e, rq.pr.s, rq.pr.d)
                    : build_graph_from_edges(rq.pr.v, rq.pr.edges, rq.pr.d));

            auto rc = std::make_shared<ResultCollector>();
            rc->fd = rq.fd;
            rc->remaining.store(4);
            {
                std::lock_guard<std::mutex> lk(rc->m);
                rc->out << "Graph:\n"
                        << gptr->getGraph() << "\nResults:\n";
            }

            for (int id = 1; id <= 4; ++id)
            {
                qRun.push(RunTask{rc, gptr, id});
            }
        }
        catch (const std::exception &ex)
        {
            send_all(rq.fd, std::string("ERROR: ") + ex.what() + "\n");
            ::close(rq.fd);
        }
    }
}

/**
 * @brief Run a single algorithm on a shared Graph.
 *
 * Executes one algorithm (given by alg_id) on the shared Graph. Appends
 * results into the shared ResultCollector buffer. When the last algorithm
 * finishes (remaining == 0), enqueues the combined Result to the send stage.
 *
 * @param qRun   Input queue of RunTasks.
 * @param qRun2S Output queue of final Results (ready to send).
 */
static void stage_run(BoundedQueue<RunTask> &qRun, BoundedQueue<Result> &qRun2S)
{
    RunTask t;
    while (qRun.pop(t))
    {
        std::string chunk;
        try
        {
            if ((t.alg_id == 2 || t.alg_id == 3) && !t.g->isDirected())
            {
                chunk = "Algorithm " + std::to_string(t.alg_id) +
                        ": Requires directed graph\n\n";
            }
            else
            {
                std::unique_ptr<Algorithm> alg(AlgorithmFactory::create(t.alg_id));
                if (alg)
                {
                    chunk = alg->run(*t.g) + std::string("\n\n");
                }
            }
        }
        catch (const std::exception &ex)
        {
            chunk = "Algorithm " + std::to_string(t.alg_id) +
                    " Error: " + ex.what() + "\n\n";
        }

        // Append the chunk to the ResultCollector's output
        {
            std::lock_guard<std::mutex> lk(t.rc->m);
            t.rc->out << chunk;
        }

        // If this was the last algorithm to finish, push the final result to the send stage
        if (t.rc->remaining.fetch_sub(1) == 1)
        {
            qRun2S.push(Result{t.rc->fd, t.rc->out.str()});
        }
    }
}

/**
 * @brief Send the final combined result to the client.
 *
 * Sends the aggregated output stored in ResultCollector to the client
 * socket, then closes the connection.
 *
 * @param qRun2S Input queue of completed Results.
 */
static void stage_send(BoundedQueue<Result> &qRun2S)
{
    Result r;
    while (qRun2S.pop(r))
    {
        send_all(r.fd, r.out);
        ::close(r.fd);
    }
}

// ==========================
// Main
// ==========================

/**
 * @brief Signal handler for SIGINT to initiate graceful shutdown.
 *
 * Sets the global stop flag and shuts down the listening socket to
 * interrupt any blocking accept() call.
 *
 */
static void on_sigint(int)
{
    g_stop.store(true);
    // Interrupt the accept() call by shutting down the socket
    if (sfd != -1)
    {
        ::shutdown(sfd, SHUT_RDWR);
    }
}

/**
 * @brief Server entry point.
 *
 * Command-line options:
 * -t <threads> Number of worker threads (default 10).
 *
 * Initializes the server, creates worker threads, and listens for incoming
 * connections. Implements graceful shutdown on SIGINT.
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
    while ((opt = ::getopt(argc, argv, "t:")) != -1)
    {
        switch (opt)
        {
        case 't':
            num_threads = std::atoi(optarg);
            if (num_threads <= 0)
            {
                std::cerr << "Number of threads must be positive\n";
                return 1;
            }
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " -t <threads>\n";
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

    // ==========================
    // Pipeline queues and threads
    // ==========================

    // ---- Pipeline queues between stages ----

    // Sizes chosen to provide some buffering while limiting memory usage
    BoundedQueue<Conn> qA2R(std::max(64, 16 * num_threads));
    BoundedQueue<RecvBuf> qR2P(std::max(64, 16 * num_threads));
    BoundedQueue<Req> qP2B(std::max(64, 8 * num_threads));
    BoundedQueue<RunTask> qRun(std::max(32, 4 * num_threads));
    BoundedQueue<Result> qRun2S(std::max(64, 16 * num_threads));

    // ---- Pipeline threads per stage ----

    // Distribute threads among stages, ensuring at least one thread per stage
    int nRecv = std::max(2, num_threads / 2);
    int nParse = std::max(2, num_threads / 2);
    int nBuild = std::max(1, num_threads / 3);
    int nRun = std::max(1, num_threads / 3);
    int nSend = std::max(2, num_threads / 2);

    // ---- Spawner helper ----
    /**
     * @brief Spawn 'n' identical worker threads running the given callable.
     * 
     * @param n Number of threads to spawn.
     * @param fn Callable to run in each thread.
     * @return Vector of joinable threads.
     */
    auto spawn = [](int n, auto &&fn)
    {
        std::vector<std::thread> v;
        v.reserve(n);
        for (int i = 0; i < n; ++i)
            v.emplace_back(fn);
        return v;
    };

    // ---- Launch stages ----
    std::thread acceptor([&]
                         { stage_accept(sfd, qA2R); });
    auto recv_threads = spawn(nRecv, [&]
                              { stage_recv(qA2R, qR2P); });
    auto parse_threads = spawn(nParse, [&]
                               { stage_parse(qR2P, qP2B); });
    auto build_threads = spawn(nBuild, [&]
                               { stage_build(qP2B, qRun); });
    auto run_threads = spawn(nRun, [&]
                             { stage_run(qRun, qRun2S); });
    auto send_threads = spawn(nSend, [&]
                              { stage_send(qRun2S); });

    std::cout << "[MultiThreadServer] Listening on port " << port << "\n"
              << "threads per stage: \n"
              << "Accept:1, Recv:" << nRecv << ", Parse:" << nParse
              << ", Build:" << nBuild << ", Run:" << nRun << ", Send:" << nSend << "\n"
              << "(Ctrl+C to stop)\n";

    // Wait for shutdown signal; pipeline threads handle all work
    while (!g_stop.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ---- Graceful shutdown ----
    if (sfd != -1)
    {
        ::close(sfd);
        sfd = -1;
    }
    qA2R.close();
    qR2P.close();
    qP2B.close();
    qRun.close();
    qRun2S.close();

    if (acceptor.joinable())
        acceptor.join();
    for (auto &t : recv_threads)
        if (t.joinable())
            t.join();
    for (auto &t : parse_threads)
        if (t.joinable())
            t.join();
    for (auto &t : build_threads)
        if (t.joinable())
            t.join();
    for (auto &t : run_threads)
        if (t.joinable())
            t.join();
    for (auto &t : send_threads)
        if (t.joinable())
            t.join();

    std::cout << "[Multithread] Stopped.\n";
    return 0;
}
