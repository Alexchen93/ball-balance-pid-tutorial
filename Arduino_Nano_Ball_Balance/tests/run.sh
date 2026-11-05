#!/usr/bin/env bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_BIN="$(mktemp)"
trap 'rm -f "$TEST_BIN"' EXIT
g++ -std=c++11 -Wall -Wextra -I "$TEST_DIR/stubs" "$TEST_DIR/test_pid.cpp" -o "$TEST_BIN"
"$TEST_BIN"
