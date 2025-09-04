CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2  -MMD -MP

SERVER_SRCS = Graph.cpp server.cpp
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

clean:
	rm -f server client $(SERVER_OBJS) $(CLIENT_OBJS) \
	      $(SERVER_OBJS:.o=.d) $(CLIENT_OBJS:.o=.d)

-include $(SERVER_OBJS:.o=.d) $(CLIENT_OBJS:.o=.d)

.PHONY: all clean
