cmake_minimum_required(VERSION 3.10)
project(ChladniSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────

# OpenGL
find_package(OpenGL REQUIRED)

# Threads
find_package(Threads REQUIRED)

# Global Defines for OpenGL Loading (Linux/X11)
add_definitions(-DGL_GLEXT_PROTOTYPES)

# ── Third Party ──────────────────────────────────────────────────────────────

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# Eigen3 (Header only, local)
include_directories(${THIRD_PARTY_DIR}/eigen)

# Nlohmann JSON (Header only, local)
include_directories(${THIRD_PARTY_DIR})

# GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)

add_subdirectory(${THIRD_PARTY_DIR}/glfw)
include_directories(${THIRD_PARTY_DIR}/glfw/include)

# ImGui
include_directories(${THIRD_PARTY_DIR})

set(IMGUI_SOURCES
    ${THIRD_PARTY_DIR}/imgui.cpp
    ${THIRD_PARTY_DIR}/imgui_draw.cpp
    ${THIRD_PARTY_DIR}/imgui_tables.cpp
    ${THIRD_PARTY_DIR}/imgui_widgets.cpp
    ${THIRD_PARTY_DIR}/imgui_impl_glfw.cpp
    ${THIRD_PARTY_DIR}/imgui_impl_opengl3.cpp
)

set(IMPLOT_SOURCES
    ${THIRD_PARTY_DIR}/implot.cpp
    ${THIRD_PARTY_DIR}/implot_items.cpp
)

# ── Project Structure ────────────────────────────────────────────────────────

include_directories(src)

set(CORE_SOURCES
    src/core/physics.cpp
    src/core/analyzer.cpp
)

set(GUI_SOURCES
    src/gui/application.cpp
    src/gui/panels.cpp
)

set(ALL_SOURCES
    src/main.cpp
    ${CORE_SOURCES}
    ${GUI_SOURCES}
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

add_executable(chladni_sim ${ALL_SOURCES})

target_link_libraries(chladni_sim
    glfw
    ${OPENGL_LIBRARIES}
    Threads::Threads
    dl
)

# ── Compiler Settings ────────────────────────────────────────────────────────

if(MSVC)
    target_compile_options(chladni_sim PRIVATE /W4)
else()
    target_compile_options(chladni_sim PRIVATE -Wall -Wextra -Wpedantic -O3)
endif()
