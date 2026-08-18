#!/usr/bin/env bash
set -euo pipefail

# Build the program in src/ by default. Pass another CMake target name to build
# one of the existing simulations instead.
target="${1:-rocket_main}"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target "${target}" --parallel
