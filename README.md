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

- **C++17** compatible compiler.
- **Eigen3**
- **nlohmann/json**
- **GLFW 3.3+**
- **OpenGL 3.3+**
- **Dear ImGui & ImPlot** (Included in `third_party/`)
