

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

static std::string recv_all_cli(int fd)
{
    std::string out;
    char buf[4096];
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0)
        out.append(buf, buf + n);
    return out;
}

int main(int argc, char **argv)
{
    std::string host = "127.0.0.1";
    int port = 8080;
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

    std::ostringstream req;
    req << vertices << ' ' << edges << ' ' << seed << "\n";

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

    std::cout << resp;
    return 0;
}
