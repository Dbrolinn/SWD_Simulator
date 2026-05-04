/**
 * @file physics.h
 * @brief This file handles the physical simulation of plate resonance modes.
 */

#pragma once

#include <complex>
#include <vector>
#include <memory>
#include <optional>
#include <mutex>
#include <Eigen/Dense>

#include "material.h"

namespace chladni {

const double kMinClearanceMeters = 0.055;
const double kGForce = 9.81;

enum class Geometry { kSquare, kRectangular, kCircular };

struct Transducer {
  double x;
  double y;
  double amplitude; // Power in Watts
  double phase_rad;
  ::std::optional<double> frequency;
};

struct SimulationContext {
  Geometry geometry;
  double lx;
  double ly;
  double h;
  double e;
  double rho;
  double nu;
  double damping;
  int n_modes;
  int sign;
  double base_volume_1 = 1.0;
  double base_volume_2 = 1.0;
  ::std::vector<Transducer> transducers;
  VibrationSpeaker speaker;
};

class PhysicsEngine {
 public:
  explicit PhysicsEngine(int resolution = 200);

  double calculate_mode_frequency(int n, int m, const SimulationContext& ctx);
  Eigen::MatrixXcd compute_driven_response(double frequency, const SimulationContext& ctx);
  void compute_visuals(double frequency, const SimulationContext& ctx, 
                       Eigen::MatrixXd& sand, Eigen::MatrixXd& deformation);

  bool check_power_feasibility(double frequency, const SimulationContext& ctx);
  
  ::std::vector<double> get_resonant_frequencies(const SimulationContext& ctx);
  ::std::vector<double> calculate_spectrum(double f_min, double f_max, int n_points, const SimulationContext& ctx);

  static double transducer_coupling_single(double tx, double ty, int n, int m, double lx, double ly, int sign);

  // ── Particle System ──────────────────────────────────────────────────
  void init_particles(int n_particles, double lx, double ly);
  void step_particles(const Eigen::MatrixXcd& response, double lx, double ly, double dt);
  const Eigen::MatrixXd& get_particles() const { return particles_; }

  /**
   * @brief Checks if a mode (n, m) is degenerate with (m, n).
   */
  bool is_degenerate(int n, int m, const SimulationContext& ctx);

  /**
   * @brief Snipes phases for a mode, handling quadrature if degenerate.
   */
  ::std::vector<double> snipe_phases(int n, int m, const SimulationContext& ctx);

  /**
   * @brief Strictly validates power feasibility (acceleration >= 9.81 m/s^2).
   */
  bool validate_power(double frequency, const SimulationContext& ctx);

  // ── Helpers ──────────────────────────────────────────────────────────
  void clamp_transducer(Transducer& t, const SimulationContext& ctx);

 private:
  int resolution_;
  Eigen::MatrixXd x_norm_;
  Eigen::MatrixXd y_norm_;
  Eigen::MatrixXd particles_;
  Eigen::MatrixXd particle_vel_;
  mutable ::std::recursive_mutex mutex_;

  static const ::std::vector<double> kFreeBeamRoots;

  struct ModalCache {
    Eigen::VectorXd omega_nm;
    Eigen::VectorXd f_nm;
    ::std::vector<Eigen::MatrixXd> modes;
    ::std::vector<int> ns;
    ::std::vector<int> ms;
    double last_lx = 0.0, last_ly = 0.0, last_h = 0.0;
    double last_e = 0.0, last_rho = 0.0, last_nu = 0.0;
    int last_n_modes = 0;
  } cache_;

  void build_cache_rect(const SimulationContext& ctx);
  void build_cache_circ(const SimulationContext& ctx);
  double get_beam_root(int n);
  Eigen::VectorXd free_beam_mode(int n, const Eigen::VectorXd& x, double L);
  double speaker_drive_gain(double freq, const SimulationContext& ctx, bool apply_compensation);
};

} // namespace chladni
