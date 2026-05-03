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
#include <Eigen/Dense>

#include "physics.h"

namespace chladni {

struct LayoutResult {
  int alphabet_size;
  double avg_quality;
  double feasibility_rate;
  ::std::vector<Transducer> best_layout;
  ::std::string layout_type;
  double param1, param2;
};

struct GAParams {
    int population_size = 50;
    int generations = 20;
    double mutation_rate = 0.15;
    int transducer_count = 4;
    ::std::string export_path = "ga_optimal_layouts.csv";
};

class Analyzer {
 public:
  explicit Analyzer(::std::shared_ptr<PhysicsEngine> physics);

  /**
   * @brief Stage 2: Genetic Algorithm Placement Optimizer.
   */
  ::std::vector<LayoutResult> run_genetic_algorithm(const SimulationContext& base_ctx, const GAParams& params);

  /**
   * @brief Stage 3: Variable Sensitivity Sweep (Phase & Amplitude).
   */
  ::std::vector<LayoutResult> run_sensitivity_sweep(const SimulationContext& base_ctx);

  // ── Existing Grid Search (Legacy fallback) ──────────────────────────
  ::std::vector<LayoutResult> run_grid_search(const SimulationContext& base_ctx);

  // ── GA Internal Operators ───────────────────────────────────────────
  void repair_layout(::std::vector<Transducer>& layout, const SimulationContext& ctx);
  ::std::vector<Transducer> crossover(const ::std::vector<Transducer>& p1, const ::std::vector<Transducer>& p2);
  void mutate(::std::vector<Transducer>& layout, double rate, const SimulationContext& ctx);

 private:
  ::std::shared_ptr<PhysicsEngine> physics_;
  ::std::mutex results_mutex_;

  bool validate_layout(const ::std::vector<Transducer>& layout);
  double calculate_similarity(const Eigen::VectorXcd& sig1, const Eigen::VectorXcd& sig2);
  LayoutResult evaluate_layout(const ::std::vector<Transducer>& layout, const SimulationContext& base_ctx);
  
  void export_to_csv(const ::std::string& path, const ::std::vector<LayoutResult>& results);
};

} // namespace chladni
