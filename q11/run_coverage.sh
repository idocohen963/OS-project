#!/usr/bin/env bash
# Script to achieve maximum code coverage for Q11 server/client
#./run_coverage.sh

set -euo pipefail

echo "=== Q11 Code Coverage Test Runner ==="

# 1) Clean and build with coverage
echo "Building with coverage flags..."


# 2) Start server in background
echo "Starting server on port 8080..."
./server -t 8  &
SERVER_PID=$!
sleep 1

echo "Server PID: $SERVER_PID"

# 3) Run client tests to maximize coverage

echo "--- Testing valid undirected connected graph (MST + CliqueCount) ---"
printf "0 1\n1 2\n0 2\n" | ./client --manual -v 3 -e 3

echo "--- Testing undirected disconnected graph (MST error path) ---"
printf "0 1\n" | ./client --manual -v 4 -e 1

echo "--- Testing directed graph (SCC + MaxFlow + CliqueCount) ---"
printf "0 1\n1 0\n1 2\n2 3\n3 2\n" | ./client --manual -v 4 -e 5 -d

echo "--- Testing MANUAL parse error: not enough edges ---"
printf "0 1\n" | ./client --manual -v 3 -e 2 || true

echo "--- Testing MANUAL parse error: vertex out of range ---"
printf "0 5\n" | ./client --manual -v 3 -e 1 || true

echo "--- Testing MANUAL build error: self-loop ---"
printf "0 0\n" | ./client --manual -v 2 -e 1 || true

echo "--- Testing MANUAL build error: duplicate edge ---"
printf "0 1\n0 1\n" | ./client --manual -v 3 -e 2 || true

echo "--- Testing RANDOM with invalid parameters (v=0) ---"
./client --random -v 0 -e 1 -s 1 || true

echo "--- Testing RANDOM with too many edges ---"
./client --random -v 3 -e 10 -s 7 -d || true

echo "--- Testing client argument errors ---"
./client --random --manual -v 3 -e 3 -s 1 || true  # both modes
./client -v 3 -e 3 || true  # no mode specified
./client --random -v 5 -e 3 || true  # missing -s
./client --manual -v 5 || true  # missing -e
./client -x || true  # unknown option

echo "--- Testing valid RANDOM graphs ---"
./client --random -v 5 -e 6 -s 42
./client --random -v 6 -e 8 -s 99 -d

echo "--- Testing edge cases for better Graph.cpp coverage ---"
# Large disconnected graph to test more paths
printf "0 1\n2 3\n4 5\n" | ./client --manual -v 10 -e 3 || true
# Single vertex graph
printf "" | ./client --manual -v 1 -e 0 || true
# Two vertices, one edge (minimal graph)
printf "0 1\n" | ./client --manual -v 2 -e 1

echo "--- Testing concurrent clients (2 simultaneous) ---"
printf "0 1\n1 2\n0 2\n" | ./client --manual -v 3 -e 3 &
CLIENT1_PID=$!
printf "0 1\n1 0\n1 2\n2 3\n3 2\n" | ./client --manual -v 4 -e 5 -d &
CLIENT2_PID=$!
wait $CLIENT1_PID || true
wait $CLIENT2_PID || true

echo "--- All client tests completed ---"

# 4) Graceful shutdown
echo "Shutting down server..."
kill -INT $SERVER_PID
wait $SERVER_PID || true

