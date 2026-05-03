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
#include <nlohmann/json.hpp>

#include "physics.h"

namespace chladni {

/**
 * @struct LayoutResult
 * @brief Container for the results of a layout evaluation.
 */
struct LayoutResult {
  int alphabet_size;
  double avg_quality;
  double feasibility_rate;
  ::std::vector<Transducer> best_layout;
  ::std::string layout_type;
  double param1, param2;
};

/**
 * @struct GAParams
 * @brief Hyperparameters for the Genetic Algorithm optimizer.
 */
struct GAParams {
  int population_size = 50;
  int generations = 20;
  double mutation_rate = 0.15;
  int transducer_count = 4;
  ::std::string export_path = "ga_optimal_layouts.json";
};

/**
 * @class Analyzer
 * @brief Orchestrates the optimization and analysis workflows.
 */
class Analyzer {
 public:
  explicit Analyzer(::std::shared_ptr<PhysicsEngine> physics);

  /**
   * @brief Stage 2: Genetic Algorithm Placement Optimizer.
   * @param base_ctx Current simulation context.
   * @param params GA hyperparameters.
   * @return A list of optimized layout results.
   */
  ::std::vector<LayoutResult> run_genetic_algorithm(const SimulationContext& base_ctx, const GAParams& params);

  /**
   * @brief Stage 3: Variable Sensitivity Sweep (Phase & Amplitude).
   * @param base_ctx Current simulation context.
   * @return A list of valid resonance modes discovered for the layout.
   */
  ::std::vector<LayoutResult> run_sensitivity_sweep(const SimulationContext& base_ctx);

  /**
   * @brief Legacy grid search fallback.
   * @param base_ctx Current simulation context.
   * @return A list of layout results.
   */
  ::std::vector<LayoutResult> run_grid_search(const SimulationContext& base_ctx);

  /**
   * @brief Enforces physical constraints on a layout.
   * @param layout The layout to repair.
   * @param ctx Current simulation context.
   */
  void repair_layout(::std::vector<Transducer>& layout, const SimulationContext& ctx);
  
  /**
   * @brief Performs crossover operator for GA.
   */
  ::std::vector<Transducer> crossover(const ::std::vector<Transducer>& p1, const ::std::vector<Transducer>& p2);
  
  /**
   * @brief Performs mutation operator for GA.
   */
  void mutate(::std::vector<Transducer>& layout, double rate, const SimulationContext& ctx);

  /**
   * @brief Stage 3/4: Exports results to JSON format.
   * @param path File system path for the export.
   * @param results The results to serialize.
   */
  void export_to_json(const ::std::string& path, const ::std::vector<LayoutResult>& results);

 private:
  ::std::shared_ptr<PhysicsEngine> physics_;
  ::std::mutex results_mutex_;

  bool validate_layout(const ::std::vector<Transducer>& layout);
  double calculate_similarity(const Eigen::VectorXcd& sig1, const Eigen::VectorXcd& sig2);
  LayoutResult evaluate_layout(const ::std::vector<Transducer>& layout, const SimulationContext& base_ctx);
};

} // namespace chladni
