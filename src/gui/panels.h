/**
 * @file panels.h
 * @brief UI panels for the Chladni simulator.
 */

#pragma once

#include <string>
#include "core/physics.h"
#include "core/analyzer.h"

namespace chladni {

class Application;

class Panels {
 public:
  static void draw_main_ui(SimulationContext& ctx, Application* app, Analyzer& analyzer, float& current_freq, float& f_max, bool& is_batch_running, float& batch_progress, ::std::string& batch_status);
  
 private:
  static void draw_stage1_manual(SimulationContext& ctx, Application* app, float& current_freq, float& f_max);
  static void draw_stage2_grid(SimulationContext& ctx, Analyzer& analyzer, Application* app);
  
  static void draw_stage3_sweep(SimulationContext& ctx, Analyzer& analyzer, float& current_freq);
  
  static void draw_stage4_batch(SimulationContext& ctx, Application* app, bool& is_batch_running, float& batch_progress, ::std::string& batch_status);
  static void draw_stage5_calibration(SimulationContext& ctx);
  
  static void draw_material_selector(SimulationContext& ctx);
};

} // namespace chladni