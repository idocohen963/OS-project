#!/bin/bash

# Minimal coverage script - only what's needed to maximize coverage

echo "=== Running coverage tests ==="

# Test 1: Basic undirected graph (typical flow)
./demo -v 10 -e 20 -s 5

# Test 2: Directed graph (covers directed-specific code paths)
./demo -v 8 -e 12 -s 10 -d

# Test 3: Empty graph (covers isConnected early return when no edges)
./demo -v 5 -e 0 -s 15

# Test 4: Guaranteed Eulerian circuit (triangle: all degrees even)
./demo -v 3 -e 3 -s 1

echo "=== Coverage tests completed ==="
