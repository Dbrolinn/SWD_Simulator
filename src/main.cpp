/**
 * @file main.cpp
 * @brief Entry point for the Chladni Plate Simulator.
 */

#include "gui/application.h"
#include <iostream>
#include <exception>

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  try {
    std::cout << "Starting SWAID Chladni Simulator..." << std::endl;
    chladni::Application app("SWAID Plate Resonance", 1280, 720);
    std::cout << "Application initialized. Running..." << std::endl;
    app.run();
    std::cout << "Application finished normally." << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "CRITICAL ERROR: Unknown exception occurred." << std::endl;
    return 1;
  }
  return 0;
}
