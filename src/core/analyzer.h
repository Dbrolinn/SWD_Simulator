/**
 * @file analyzer.h
 * @brief Multi-threaded analyzer for plate resonance optimization.
 */

#pragma once

#include <vector>
#include <future>
#include <mutex>
#include <string>
#include <memory>
#include <atomic>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "physics.h"

namespace chladni {

struct LayoutResult {
  int alphabet_size;
  double avg_quality;
  double feasibility_rate;
  ::std::vector<Transducer> best_layout;
  ::std::string layout_type;
  double param1, param2;
  double total_displacement = 0.0;
  
  float grid_dx = 0.0f;
  float grid_dy = 0.0f;
  
  float achieved_g = 0.0f;
  float required_power_w = 0.0f;
  float required_digital_amp = 0.0f;
  bool is_clipping = false;

  bool export_selected = true;
};

struct GridParams {
  bool use_roi = false;
  float roi_dx_min = 0.0f;
  float roi_dx_max = 0.0f;
  float roi_dy_min = 0.0f;
  float roi_dy_max = 0.0f;

  float start_offset_m = 0.025f;
  float step_size_m = 0.015f; 
  float edge_gap_m = 0.025f;
};

struct HeatmapData {
    ::std::vector<float> scores;
    int nx = 0;
    int ny = 0;
    float dx_min = 0.0f;
    float dx_max = 0.0f;
    float dy_min = 0.0f;
    float dy_max = 0.0f;
    bool valid = false;
};

class Analyzer {
 public:
  explicit Analyzer(::std::shared_ptr<PhysicsEngine> physics);

  ::std::vector<LayoutResult> run_symmetric_grid_search(const SimulationContext& base_ctx, const GridParams& params);
  ::std::vector<LayoutResult> run_sensitivity_sweep(const SimulationContext& base_ctx);
  ::std::vector<LayoutResult> run_grid_search(const SimulationContext& base_ctx);

  void repair_layout(::std::vector<Transducer>& layout, const SimulationContext& ctx);
  void export_to_json(const ::std::string& path, const ::std::vector<LayoutResult>& results, const SimulationContext& ctx);

  const HeatmapData& get_heatmap_data() const { return heatmap_; }
  bool load_sim_results(const ::std::string& filepath, SimulationContext& ctx);
  
  const ::std::vector<LayoutResult>& get_top_layouts() const { return top_layouts_; }
  void clear_heatmap() { heatmap_.valid = false; heatmap_.scores.clear(); top_layouts_.clear(); }

  const ::std::vector<LayoutResult>& get_sweep_results() const { return sweep_results_; }
  void load_sweep_results(const ::std::string& filepath, SimulationContext& ctx);

  float get_grid_progress() const { return grid_progress_.load(); }
  float get_grid_sub_progress() const { 
      return expected_modes_.load() > 0 ? (float)modes_evaluated_.load() / expected_modes_.load() : 0.0f; 
  }
  float get_sweep_progress() const { return sweep_progress_.load(); }
  int get_grid_best_alphabet() const { return grid_best_alphabet_.load(); }
  bool is_analyzing() const { return is_analyzing_.load(); }

 private:
  ::std::shared_ptr<PhysicsEngine> physics_;
  ::std::mutex results_mutex_;
  
  HeatmapData heatmap_;
  ::std::vector<LayoutResult> top_layouts_;
  ::std::vector<LayoutResult> sweep_results_;

  ::std::atomic<float> grid_progress_{0.0f};
  ::std::atomic<int> modes_evaluated_{0};
  ::std::atomic<int> expected_modes_{1};
  ::std::atomic<float> sweep_progress_{0.0f};
  ::std::atomic<int> grid_best_alphabet_{0};
  ::std::atomic<bool> is_analyzing_{false};

  bool validate_layout(const ::std::vector<Transducer>& layout);
  LayoutResult evaluate_layout(const ::std::vector<Transducer>& layout, const SimulationContext& base_ctx);
  
  void auto_export_grid_results(const SimulationContext& ctx);
  void auto_export_sweep_results(const SimulationContext& ctx);
};

} // namespace chladni