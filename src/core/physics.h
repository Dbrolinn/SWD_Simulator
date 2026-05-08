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

const double kGForce = 9.81;

enum class Geometry { kSquare, kRectangular, kCircular };

struct Transducer {
  double x;
  double y;
  double amplitude; // Now acts purely as the Software Digital Amp (0.0 to 1.0)
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
  double max_frequency = 20000.0;
  int sign;
  
  // Frequency Calibration
  double calib_m = 1.0;
  double calib_b = 0.0;
  
  // Empirical Power Curve Calibration (Power_1G = A*f^2 + B*f + C)
  double p_A = 0.0;
  double p_B = 0.0;
  double p_C = 1.0;
  
  double transducer_radius_m = 0.025;
  double transducer_spacing_m = 0.005;

  // Auto-Tuner Target Goals
  double transducer_max_power_w = 25.0;
  double hardware_amp_gain = 0.8;
  double target_g_force = 5.0;
  double particle_mass_mg = 1.0;

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

  bool is_degenerate(int n, int m, const SimulationContext& ctx);
  ::std::vector<double> snipe_phases(int n, int m, const SimulationContext& ctx);
  bool validate_power(double frequency, const SimulationContext& ctx);
  void clamp_transducer(Transducer& t, const SimulationContext& ctx);

  int get_resolution() const { return resolution_; }

 private:
  int resolution_;
  Eigen::MatrixXd x_norm_;
  Eigen::MatrixXd y_norm_;
  Eigen::MatrixXd particles_;
  Eigen::MatrixXd particle_vel_;
  mutable ::std::recursive_mutex mutex_;

  struct ResponseCache {
      double last_f = -1.0;
      Eigen::MatrixXcd last_resp;
      ::std::vector<Transducer> last_transducers;
      double last_hw_gain = -1.0;
      double last_damping = -1.0;
  } resp_cache_;

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