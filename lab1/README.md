# Lab 1 — Random Vector (C++ basics)

Standalone C++ exercise: implement `RandomVector` (fill, print, mean/min/max, ASCII histogram). No ROS launch file.

## Quick start (Docker)

```bash
bash ros2-docker/run_lab_dev.sh lab1
```

That opens a shell in the lab container. Then:

```bash
cd /workspace/VNAV-labs/lab1
cmake -S . -B build
cmake --build build
./build/lab1
```

Build only from the host (no interactive shell):

```bash
bash ros2-docker/run_lab_dev.sh lab1 --build-only
```

`colcon` looks for package `lab1_random_vector`. If that target is missing, use the CMake commands above.

## Local (no Docker)

```bash
cd VNAV-labs/lab1
cmake -S . -B build
cmake --build build
./build/lab1
```

Requires CMake ≥ 3.16 and a C++17 compiler. `main.cpp` seeds `std::srand(314159)` so output is reproducible.

## Key files

| File | Purpose |
|------|---------|
| `random_vector.h` | `RandomVector` interface |
| `random_vector.cpp` | Implementation (fill, stats, histogram) |
| `main.cpp` | Driver: size 20, print stats + 5-bin histogram |
| `CMakeLists.txt` | Builds executable `lab1` |
