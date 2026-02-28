#!/bin/bash
# ===============================================
# verify_all.sh - Verification Main Script
# ===============================================
set -e

echo "=== PID & Kalman Filter PoC Verification ==="
echo ""

# Create directories
mkdir -p build tests/output

# Compile all modules
echo "[1/5] Compiling modules..."
gcc -Wall -Wextra -O2 -c pid.c -o build/pid.o
gcc -Wall -Wextra -O2 -c kalman.c -o build/kalman.o
gcc -Wall -Wextra -O2 -c ekf_attitude.c -o build/ekf.o -lm
echo "  [OK] Compilation successful"

# Test 1: PID unit tests
echo ""
echo "[2/5] PID Unit Tests..."
gcc -Wall -O2 -o build/test_pid tests/test_pid.c pid.c -lm
./build/test_pid
echo "  [OK] PID tests passed"

# Test 2: Kalman Filter unit tests
echo ""
echo "[3/5] Kalman Filter Unit Tests..."
gcc -Wall -O2 -o build/test_kalman tests/test_kalman.c kalman.c -lm
./build/test_kalman
echo "  [OK] Kalman Filter tests passed"

# Test 3: Integration simulation
echo ""
echo "[4/5] Kalman + PID Integration Simulation..."
gcc -Wall -O2 -o build/test_integration tests/test_integration.c pid.c kalman.c -lm
./build/test_integration > tests/output/integration_results.csv
echo "  [OK] Integration simulation done -> tests/output/integration_results.csv"

# Test 4: EKF test
echo ""
echo "[5/5] EKF Attitude Estimation Tests..."
gcc -Wall -O2 -o build/test_ekf tests/test_ekf.c ekf_attitude.c -lm
./build/test_ekf
echo "  [OK] EKF tests passed"

echo ""
echo "=== All verifications completed ==="
