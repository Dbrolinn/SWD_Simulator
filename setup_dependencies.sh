#!/bin/bash

# SWAID Chladni Simulator - Dependency Setup Script
# This script populates the third_party directory with required libraries.

set -e

PROJECT_ROOT=$(pwd)
THIRD_PARTY_DIR="$PROJECT_ROOT/third_party"

echo "Creating third_party directory..."
mkdir -p "$THIRD_PARTY_DIR"

# 1. GLFW
echo "Fetching GLFW..."
if [ ! -d "$THIRD_PARTY_DIR/glfw" ]; then
    git clone --depth 1 https://github.com/glfw/glfw.git "$THIRD_PARTY_DIR/glfw"
else
    echo "GLFW already exists, skipping clone."
fi

# 2. Dear ImGui (Docking Branch)
echo "Fetching Dear ImGui (docking branch)..."
git clone --depth 1 -b docking https://github.com/ocornut/imgui.git "$THIRD_PARTY_DIR/imgui_repo"
cp "$THIRD_PARTY_DIR/imgui_repo"/*.cpp "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo"/*.h "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo/backends/imgui_impl_glfw.cpp" "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo/backends/imgui_impl_glfw.h" "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo/backends/imgui_impl_opengl3.cpp" "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo/backends/imgui_impl_opengl3.h" "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/imgui_repo/backends/imgui_impl_opengl3_loader.h" "$THIRD_PARTY_DIR/"
rm -rf "$THIRD_PARTY_DIR/imgui_repo"

# 3. ImPlot
echo "Fetching ImPlot..."
git clone --depth 1 https://github.com/epezent/implot.git "$THIRD_PARTY_DIR/implot_repo"
cp "$THIRD_PARTY_DIR/implot_repo"/*.cpp "$THIRD_PARTY_DIR/"
cp "$THIRD_PARTY_DIR/implot_repo"/*.h "$THIRD_PARTY_DIR/"
rm -rf "$THIRD_PARTY_DIR/implot_repo"

# 4. STB Headers
echo "Fetching STB headers..."
curl -L https://raw.githubusercontent.com/nothings/stb/master/stb_image.h -o "$THIRD_PARTY_DIR/stb_image.h"
curl -L https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h -o "$THIRD_PARTY_DIR/stb_image_write.h"

# 5. Nlohmann JSON
echo "Fetching Nlohmann JSON..."
mkdir -p "$THIRD_PARTY_DIR/nlohmann"
if [ ! -f "$THIRD_PARTY_DIR/nlohmann/json.hpp" ]; then
    curl -L https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o "$THIRD_PARTY_DIR/nlohmann/json.hpp"
else
    echo "Nlohmann JSON already exists, skipping download."
fi

# 6. Eigen3
echo "Fetching Eigen3..."
if [ ! -d "$THIRD_PARTY_DIR/eigen" ]; then
    git clone --depth 1 -b 3.4.0 https://gitlab.com/libeigen/eigen.git "$THIRD_PARTY_DIR/eigen"
else
    echo "Eigen already exists, skipping clone."
fi

# 7. Build Directory
echo "Creating build directory..."
mkdir -p "$PROJECT_ROOT/build"

echo "--------------------------------------------------"
echo "Setup complete! To build the project, run:"
echo "  cd build"
echo "  cmake .."
echo "  make"
echo "--------------------------------------------------"
