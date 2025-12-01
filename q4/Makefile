CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
SRCS = Graph.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = demo

# Coverage flags
COVERAGE_FLAGS = -fprofile-arcs -ftest-coverage
COVERAGE_LDFLAGS = -lgcov

# Gprof flags
GPROF_FLAGS = -pg

all: $(TARGET)
	./demo -v 10 -e 20 -s 5

$(TARGET): Graph.o main.o
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^

%.o: %.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


VALGRIND_ARGS = -v 3 -e 2 -s 5

results-dir:
	mkdir -p results

valgrind: $(TARGET) results-dir
	valgrind --leak-check=full ./$(TARGET) $(VALGRIND_ARGS) > results/valgrind_output.txt 2>&1

callgrind: $(TARGET) results-dir
	valgrind --tool=callgrind --callgrind-out-file=results/callgrind.out --simulate-cache=yes ./$(TARGET) $(VALGRIND_ARGS)

gprof: clean results-dir
	$(CXX) $(CXXFLAGS) $(GPROF_FLAGS) $(SRCS) -o $(TARGET)
	./$(TARGET) -v 10000 -e 2500000 -s 5
	gprof $(TARGET) gmon.out > results/gprof_report.txt

coverage: clean results-dir
	$(CXX) $(CXXFLAGS) $(COVERAGE_FLAGS) -c Graph.cpp -o Graph.o
	$(CXX) $(CXXFLAGS) $(COVERAGE_FLAGS) -c main.cpp -o main.o
	$(CXX) $(CXXFLAGS) $(COVERAGE_FLAGS) Graph.o main.o -o $(TARGET) $(COVERAGE_LDFLAGS)
	./run_coverage.sh
	gcov $(SRCS)
	mkdir -p results_gcov
	mv *.gcov results_gcov/

clean:
	rm -f $(OBJS) $(TARGET) gmon.out *.gcda *.gcno *.gcov
	

.PHONY: all clean coverage valgrind callgrind gprof
