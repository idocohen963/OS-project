// ==========================
// File: server.cpp 
// Multithreaded TCP server using Leader/Followers pattern
// Receives a graph (or parameters for a random graph) and runs all 4 algorithms
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

struct ParsedRequest
{
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
            err = "Usage: RANDOM <vertices> <edges> <seed> [directed]\n";
            return std::nullopt;
        }

        int dirFlag;
        if (in >> dirFlag)
        {
            r.d = (dirFlag != 0);
        }

        // לא צריך יותר לקרוא אלגוריתם - נריץ את כולם
        // int alg_id = 0;
        // if (!read_required_alg(in, alg_id, err))
        //     return std::nullopt;

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

        // לא צריך יותר לקרוא אלגוריתם - נריץ את כולם
        // int alg_id = 0;
        // if (!read_required_alg(in, alg_id, err))
        //     return std::nullopt;

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

// ==========================
// Handle a client connection - עודכן לרוץ כל 4 האלגוריתמים
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
        out << "Graph:\n" << g.getGraph() << "\nResults:\n";

        // הרצת כל 4 האלגוריתמים בלולאה פשוטה
        for (int alg_id = 1; alg_id <= 4; ++alg_id) {
            try {
                // בדיקה אם האלגוריתם דורש גרף מכוון
                if ((alg_id == 2 || alg_id == 3) && !g.isDirected()) {
                    out << "Algorithm " << alg_id << ": Requires directed graph\n";
                    continue;
                }
                Algorithm* alg = AlgorithmFactory::create(alg_id);
                if (alg) {
                    out << "Algorithm " << alg_id << ": " << alg->run(g) << "\n";
                    delete alg;
                }
            } catch (const std::exception &ex) {
                out << "Algorithm " << alg_id << " Error: " << ex.what() << "\n";
            }
        }

        send_all(cfd, out.str());
    }
    catch (const std::exception &ex)
    {
        send_all(cfd, std::string("ERROR: ") + ex.what() + "\n");
    }
}

// ==========================
// Thread Pool globals - פשוט ויעיל לניהול threads
// ==========================

std::queue<int> client_queue;              // תור של client connections
std::mutex queue_mutex;                    // מנעול לתור
std::condition_variable queue_condition;   // סינכרון threads
std::atomic<bool> server_stop(false);     // דגל עצירה

// Worker thread function - פונקציה שרצה בכל thread
void worker_thread() {
    while (!server_stop) {
        int client_fd = -1;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_condition.wait(lock, []{ return !client_queue.empty() || server_stop; });
            
            if (server_stop && client_queue.empty()) break;
            
            if (!client_queue.empty()) {
                client_fd = client_queue.front();
                client_queue.pop();
            }
        }
        
        if (client_fd != -1) {
            handle_client(client_fd);
            ::close(client_fd);
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
    server_stop = true;  // עדכון גם ל-thread pool
    queue_condition.notify_all(); // מעורר את כל ה-threads לסיום
    if (sfd != -1)
    {
        ::close(sfd);
        sfd = -1;
    }
}

int main(int argc, char **argv)
{
    int port = 8080;
    int num_threads = 4; // מספר ברירת מחדל של threads
    int opt;
    
    // הוספת אפשרות לקבלת מספר threads מהמשתמש
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

    // יצירת worker threads - Leader/Followers pattern
    std::vector<std::thread> workers;
    for (int i = 0; i < num_threads; ++i) {
        workers.emplace_back(worker_thread);
    }

    std::cout << "[MultiThreadServer] Listening on port " << port 
              << " with " << num_threads << " threads... (Ctrl+C to stop)\n" << std::flush;
    
    // הלופ הראשי - מקבל connections ושם בתור
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
        
        // הוספת connection לתור - Leader/Followers pattern
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            client_queue.push(cfd);
        }
        queue_condition.notify_one(); // מעורר thread אחד
    }
    
    // סגירת השרת ו-threads
    server_stop = true;
    queue_condition.notify_all();
    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    
    ::close(sfd);
    std::cout << "[MultiThreadServer] Stopped.\n";
    return 0;
}
