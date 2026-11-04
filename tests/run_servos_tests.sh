#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror \
    -I"$project_dir/tests/servo_stubs" \
    -I"$project_dir/main/config" \
    -I"$project_dir/main/control" \
    -I"$project_dir/main" \
    "$project_dir/tests/servos_test.cpp" -o "$test_dir/servos_test"
"$test_dir/servos_test"
echo "Servo batch validation and five-channel motion tests passed"
