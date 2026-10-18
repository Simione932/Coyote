#!/bin/bash
set -e
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G "Ninja"
cmake --build build -j
