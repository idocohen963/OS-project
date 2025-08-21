CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
SRCS = Graph.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = demo

all: $(TARGET)
	./demo

$(TARGET): Graph.o main.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^

%.o: %.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


VALGRIND_ARGS = -v 3 -e 2 -s 5

valgrind: $(TARGET)
	valgrind --leak-check=full ./$(TARGET) $(VALGRIND_ARGS) > valgrind_output.txt 2>&1

callgrind: $(TARGET)
	valgrind --tool=callgrind --simulate-cache=yes ./$(TARGET) $(VALGRIND_ARGS)

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
