#!/bin/bash
set -e
gcc -Wall -Wextra main.c renderer.c microui.c -framework Cocoa -l c++ -o main
gcc -Wall -Wextra dglab_gui.cpp renderer.c microui.c -framework Cocoa -l c++ -o dglab
