# SWAID Chladni Simulator

A hardware-accurate plate resonance calibration suite for the SWAID Plate Resonance team.

## Overview

This simulator transitions from a theoretical sandbox into a strict calibration tool. It enforces physical hardware constraints, optimizes acoustic power requirements, and produces unified `master_symbols.json` files for the SWAID pipeline.

## Features

- **Amplitude Optimization:** Uses binary search to find the minimum power (0-25W) required to achieve 1G acceleration, preventing thermal overload.
- **Sensitivity Sweep:** Identifies actual resonance peaks around theoretical frequencies (+/- 5%).
- **JSON Pipeline:** Replaced legacy CSV format with IPC System State Bus compatible JSON serialization.
- **Hardware-Accurate UI:** Enforces SWAID plate dimensions (default 300x200mm) and transducer mechanical clearances (50mm radius).

## Workflow (The 4 Stages)

1. **Manual Stage:** Configure plate geometry and material. Manually place up to 4 transducers.
2. **GA Optimizer:** Run a Genetic Algorithm to discover the best physical layouts for a diverse "alphabet" of resonance patterns.
3. **Variable Sweep:** Perform an exhaustive sweep of the locked layout to find exact resonance peaks and optimized power settings. Export results to `master_symbols.json`.
4. **Batch Plotter:** Render Chladni figures automatically from `master_symbols.json`.

## Build Requirements

- **C++17** compatible compiler (e.g., GCC 9+, Clang 10+).
- **Eigen3**
- **nlohmann/json**
- **Git** (for fetching dependencies)
- **CMake 3.10+**

## Installation

Since third-party libraries and build artifacts are not committed to the repository, you must run the setup script first:

1. **Clone the repository:**
   ```bash
   git clone <repository_url>
   cd SWD_Simulator
   ```

2. **Install dependencies:**
   ```bash
   ./setup_dependencies.sh
   ```
   This script fetches **GLFW**, **Dear ImGui (docking)**, **ImPlot**, and **STB** into the `third_party` directory and prepares the `build` folder.

## Build & Run Guide

Follow these commands to compile the project and start the application:

### 1. Compile the Project
Navigate to the build directory and use CMake to generate the build files, then compile using `make`:
```bash
cd build
cmake ..
make
```

### 2. Run the Application
Once the build is complete, execute the simulator from the `build` directory:
```bash
./chladni_sim
```

---
**Note:** Ensure you have the system-level requirements (GCC/Clang, Eigen3, and nlohmann/json) installed before building. On Debian/Ubuntu, you can install the core dependencies with:
`sudo apt install libeigen3-dev nlohmann-json3-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev`
