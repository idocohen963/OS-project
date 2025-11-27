CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2  -MMD -MP -pthread

SERVER_SRCS = Graph.cpp server.cpp algorithms/SCC.cpp algorithms/MST.cpp algorithms/MaxFlow.cpp algorithms/CliqueCount.cpp
CLIENT_SRCS = client.cpp

SERVER_OBJS = $(SERVER_SRCS:.cpp=.o)
CLIENT_OBJS = $(CLIENT_SRCS:.cpp=.o)

all: server client

server: $(SERVER_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

client: $(CLIENT_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Valgrind targets for server
valgrind-memcheck-server: server
	mkdir -p results
	timeout 1s valgrind --leak-check=full --show-leak-kinds=all ./server 2> results/memcheck_server.txt || true

valgrind-helgrind-server: server
	mkdir -p results
	timeout 1s valgrind --tool=helgrind ./server 2> results/helgrind_server.txt || true

valgrind-cg-server: server
	mkdir -p results
	timeout 1s valgrind --tool=callgrind ./server 2> results/cg_server.txt || true

# Valgrind targets for client (with test parameters)
valgrind-memcheck-client: client server
	mkdir -p results
	./server & SERVER_PID=$$!; sleep 10; valgrind --leak-check=full --show-leak-kinds=all ./client --random -v 10 -e 30 -s 42 2> results/memcheck_client.txt; kill $$SERVER_PID 2>/dev/null || true

valgrind-helgrind-client: client server
	mkdir -p results
	./server & SERVER_PID=$$!; sleep 10; valgrind --tool=helgrind ./client --random -v 10 -e 30 -s 42 2> results/helgrind_client.txt; kill $$SERVER_PID 2>/dev/null || true

valgrind-cg-client: client server
	mkdir -p results
	./server & SERVER_PID=$$!; sleep 10; valgrind --tool=callgrind ./client --random -v 10 -e 30 -s 42 2> results/cg_client.txt; kill $$SERVER_PID 2>/dev/null || true

# Run all valgrind analyses
valgrind-all: all valgrind-memcheck-server valgrind-helgrind-server valgrind-cg-server valgrind-memcheck-client valgrind-helgrind-client valgrind-cg-client

clean:
	rm -f server client $(SERVER_OBJS) $(CLIENT_OBJS) \
		  *.d *.o algorithms/*.d algorithms/*.o \
		  callgrind.out.*

-include $(SERVER_OBJS:.o=.d) $(CLIENT_OBJS:.o=.d)

.PHONY: all clean results-dir
