#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -I"$project_dir/tests/lidar_stubs" -I"$project_dir/components/ldstl26_lidar" \
    -I"$project_dir/main/tasks/sensors" \
    "$project_dir/tests/lidar_360_test.cpp" \
    "$project_dir/components/ldstl26_lidar/ldstl26_lidar.cpp" -o "$test_dir/lidar_360_test"
"$test_dir/lidar_360_test"
