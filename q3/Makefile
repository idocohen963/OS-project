CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
SRCS = Graph.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = demo

all: $(TARGET)

$(TARGET): ${OBJS}
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^

%.o: %.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
