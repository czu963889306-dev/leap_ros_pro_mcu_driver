#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_dir="$(mktemp -d)"
trap 'rm -rf -- "$test_dir"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror \
    -I"$project_dir/tests/time_sync_stubs" \
    -I"$project_dir/main/config" \
    -I"$project_dir/main/tasks/network/microros" \
    "$project_dir/main/tasks/network/microros/microros_time_sync.cpp" \
    "$project_dir/tests/time_sync_test.cpp" -o "$test_dir/time_sync_test"
"$test_dir/time_sync_test"
