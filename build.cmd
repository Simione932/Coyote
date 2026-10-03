
call "\MinGW\set_distro_paths.bat"
rd /s/q build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles"
cmake --build build -j
