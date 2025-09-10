
/**
 * @file client.cpp
 * @brief Client for the graph server. Sends either a random-parameters request or a
 *        manual graph description and prints the server's response.
 *
 * The client supports two modes: --random and --manual. In --random mode the client
 * sends the desired number of vertices/edges and an RNG seed. In --manual mode the
 * client reads edges from stdin and forwards them to the server. The protocol is a simple
 * text protocol (see usage text below).
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <iostream>
#include <sstream>
#include <string>
#include <limits>

/**
 * @brief Send the entire buffer over a blocking socket, retrying on partial sends.
 *
 * @param fd Socket file descriptor to send on.
 * @param s String buffer to send (bytes sent are s.size()).
 * @return true if all bytes were sent, false on error.
 */
static bool send_all_cli(int fd, const std::string &s)
{
    const char *p = s.data();
    size_t left = s.size();
    while (left)
    {
        ssize_t n = ::send(fd, p, left, 0);
        if (n <= 0)
            return false;
        p += n;
        left -= n;
    }
    return true;
}

/**
 * @brief Receive all available data from a blocking socket until EOF.
 *
 * This function repeatedly calls recv(2) and appends data to a string until
 * the peer performs an orderly shutdown (recv returns 0) or an error occurs.
 *
 * @param fd Socket file descriptor to receive from.
 * @return std::string containing the received bytes (may be empty on error).
 */
static std::string recv_all_cli(int fd)
{
    std::string out;
    char buf[4096];
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0)
        out.append(buf, buf + n);
    return out;
}

/**
 * @brief Print usage information for the client.
 *
 * @param prog Program name (argv[0]) used in the usage message.
 */
static void print_usage(const char *prog)
{
    std::cerr
        << "Usage:\n"
        << "  " << prog << " --random -v <vertices> -e <edges> -s <seed> [-d]\n"
        << "  " << prog << " --manual -v <vertices> -e <edges> [-d]   <  edges on stdin\n\n"
        << "Flags:\n"
        << "  --random           Generate a random graph on server side\n"
        << "  --manual           Send a manual graph (read edges from stdin)\n"
        << "  -v <num>           Number of vertices (required in both modes)\n"
        << "  -e <num>           Number of edges (required in --random)\n"
        << "  -s <num>           Seed for RNG (required in --random)\n"
        << "  -d                 Directed graph (optional; default undirected)\n"
        << "\nNote: Server will run all 4 algorithms on the provided graph\n";
}

/**
 * @brief Program entry point for the client.
 *
 * Parses command-line arguments, constructs a textual protocol message and
 * sends it to the server; then prints the server response to stdout.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return exit status: 0 on success, non-zero on error.
 */
int main(int argc, char **argv)
{
    std::string host = "127.0.0.1";
    int port = 8080;

    enum Mode
    {
        NONE,
        RANDOM,
        MANUAL
    } mode = NONE;
    int vertices = -1, edges = -1, seed = -1;
    bool directed = false;

    // --- Pass 1: decide mode ---
    opterr = 0; // don't auto-print errors
    static option longopts[] = {
        {"random", no_argument, nullptr, 1000},
        {"manual", no_argument, nullptr, 1001},
        {"vertices", required_argument, nullptr, 'v'},
        {"edges", required_argument, nullptr, 'e'},
        {"seed", required_argument, nullptr, 's'},
        {"directed", no_argument, nullptr, 'd'},
        {nullptr, 0, nullptr, 0}};

    int c;
    while ((c = getopt_long(argc, argv, ":v:e:s:d", longopts, nullptr)) != -1)
    {
        switch (c)
        {
        case 1000: // --random
            if (mode == MANUAL)
            {
                std::cerr << "ERROR: both --random and --manual given\n";
                return 2;
            }
            mode = RANDOM;
            break;
        case 1001: // --manual
            if (mode == RANDOM)
            {
                std::cerr << "ERROR: both --random and --manual given\n";
                return 2;
            }
            mode = MANUAL;
            break;
        case 'v':
            vertices = std::stoi(optarg);
            break;
        case 'e':
            edges = std::stoi(optarg);
            break;
        case 's':
            seed = std::stoi(optarg);
            break;
        case 'd':
            directed = true;
            break;
        case ':': // missing argument
            std::cerr << "ERROR: option -" << char(optopt) << " requires an argument\n";
            print_usage(argv[0]);
            return 1;
        default:
            std::cerr << "ERROR: unknown option\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (mode == NONE)
    {
        std::cerr << "ERROR: must pass one of --random or --manual\n";
        print_usage(argv[0]);
        return 2;
    }

    std::ostringstream req;

    if (mode == RANDOM)
    {
        // Check that all required parameters were provided
        if (vertices == -1 || edges == -1 || seed == -1)
        {
            std::cerr << "Usage: " << argv[0] << " -v vertices -e edges -s seed [-d]\n";
            print_usage(argv[0]);
            return 1;
        }

        // If values are invalid
        if (vertices <= 0 || edges < 0)
        {
            std::cerr << "ERROR: vertices must be >0 and edges >=0.\n";
        }



    // Protocol for random mode - send: RANDOM <vertices> <edges> <seed> <directed_flag>\n
    req << "RANDOM" << ' ' << vertices << ' ' << edges << ' ' << seed << ' ' << directed << "\n";
    }

    else // (mode == MANUAL)
    {
        // Check that all required parameters were provided
        if (vertices == -1 || edges == -1)
        {
            std::cerr << "Usage: " << argv[0] << " -v vertices -e edges -s [-d]\n";
            print_usage(argv[0]);
            return 1;
        }

        if (vertices <= 0)
        {
            std::cerr << "ERROR: vertices must be >0.\n";
            return 1;
        }

        if (edges < 0)
        {
            std::cerr << "ERROR: edges must be >=0\n";
            return 1;
        }

        // Read edges from stdin until EOF, pass through as-is.
        // Expected per line: "u v [w]"
        std::string edges_buf;
        std::string line;
        int count = 0;

        while (count < edges && std::getline(std::cin, line))
        {
            // ignore pure empty lines in the middle to avoid accidental termination
            if (line.empty())
                continue;
            edges_buf.append(line);
            edges_buf.push_back('\n');
            ++count;
        }


        req << "MANUAL" << ' ' << vertices << ' ' << edges << ' ' << directed << "\n";
        req << edges_buf;
        req << "\n";
    }

    // --- Networking ---
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        perror("socket");
        return 1;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
    {
        std::cerr << "Bad host";
        return 1;
    }
    if (::connect(fd, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("connect");
        return 1;
    }

    std::string req_s = req.str();
    if (!send_all_cli(fd, req_s))
    {
        std::cerr << "send failed\n";
        return 1;
    }
    ::shutdown(fd, SHUT_WR);
    std::string resp = recv_all_cli(fd);
    ::close(fd);

    std::cout << resp << std::endl ;
    return 0;
}
