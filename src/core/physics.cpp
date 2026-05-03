/**
 * @file physics.cpp
 * @brief This file handles the physical simulation of plate resonance modes.
 */

#include "physics.h"
#include <cmath>
#include <algorithm>
#include <map>
#include <random>
#include <iostream>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace chladni {

const ::std::vector<double> PhysicsEngine::kFreeBeamRoots = {
    4.730040744862704, 7.853204624095838, 10.995607838001671,
    14.137165491257464, 17.278759657399482, 20.420352251041250,
    23.561944901923447, 26.703537555510351, 29.845130209103030,
    32.986722862692830, 36.128315516282620
};

PhysicsEngine::PhysicsEngine(int resolution) : resolution_(resolution) {
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(resolution, -0.5, 0.5);
  x_norm_.resize(resolution, resolution);
  y_norm_.resize(resolution, resolution);
  for (int j = 0; j < resolution; ++j) {
    x_norm_.col(j).setConstant(t[j]);
    y_norm_.row(j).setConstant(t[j]);
  }
}

double PhysicsEngine::get_beam_root(int n) {
  if (n >= 1 && n <= static_cast<int>(kFreeBeamRoots.size())) return kFreeBeamRoots[n - 1];
  return (static_cast<double>(n) + 0.5) * M_PI;
}

Eigen::VectorXd PhysicsEngine::free_beam_mode(int n, const Eigen::VectorXd& x, double L) {
  double beta_n = get_beam_root(n);
  double k_n = beta_n / ::std::max(L, 1e-12);
  double sigma_n = (::std::cosh(beta_n) - ::std::cos(beta_n)) / (::std::sinh(beta_n) - ::std::sin(beta_n));
  Eigen::VectorXd phi = (k_n * x).array().cosh() + (k_n * x).array().cos() -
                        sigma_n * ((k_n * x).array().sinh() + (k_n * x).array().sin());
  double norm = ::std::sqrt((phi.array().pow(2)).mean());
  if (norm > 0) phi /= norm;
  return phi;
}

double PhysicsEngine::speaker_drive_gain(double freq, const SimulationContext& ctx, bool apply_compensation) {
  double f = ::std::max(freq, 1e-9);
  double low = 1.0 / (1.0 + ::std::pow(ctx.speaker.f_min / f, 8));
  double high = 1.0 / (1.0 + ::std::pow(f / ctx.speaker.f_max, 8));
  double bandwidth = low * high;
  double f_ref_mass = 1200.0 / ::std::sqrt(::std::max(ctx.speaker.mass_kg, 1e-6) / 0.268);
  double mass_gain = 1.0 / ::std::sqrt(1.0 + ::std::pow(f / f_ref_mass, 2));
  double gain = bandwidth * mass_gain;
  if (apply_compensation) {
      double f_ref_comp = 50.0;
      gain *= ::std::pow(::std::max(freq, f_ref_comp) / f_ref_comp, 1.8);
  }
  return gain;
}

double PhysicsEngine::calculate_mode_frequency(int n, int m, const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  double D = (ctx.e * ::std::pow(ctx.h, 3)) / (12.0 * (1.0 - ::std::pow(ctx.nu, 2)));
  if (ctx.geometry == Geometry::kCircular) {
    static const ::std::map<::std::pair<int, int>, double> bessel_zeros = {
        {{0, 1}, 2.4048}, {{0, 2}, 5.5201}, {{0, 3}, 8.6537},
        {{1, 1}, 3.8317}, {{1, 2}, 7.0156}, {{1, 3}, 10.1735},
        {{2, 1}, 5.1356}, {{2, 2}, 8.4172}, {{2, 3}, 11.6198},
        {{3, 1}, 6.3802}, {{3, 2}, 9.7610}, {{3, 3}, 13.0152}
    };
    double R = ctx.lx / 2.0;
    if (bessel_zeros.count({n, m})) {
        double lam = bessel_zeros.at({n, m});
        return (::std::pow(lam, 2) / (2.0 * M_PI * ::std::pow(R, 2))) * ::std::sqrt(D / (ctx.rho * ctx.h));
    }
    return (n + m) * 500.0; // Fallback
  } else {
    double kx = get_beam_root(n) / ctx.lx;
    double ky = get_beam_root(m) / ctx.ly;
    double omega_nm = ::std::sqrt(D / (ctx.rho * ctx.h)) * (::std::pow(kx, 2) + ::std::pow(ky, 2));
    return omega_nm / (2.0 * M_PI);
  }
}

void PhysicsEngine::build_cache_rect(const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  if (cache_.last_lx == ctx.lx && cache_.last_ly == ctx.ly && cache_.last_n_modes == ctx.n_modes && ctx.geometry != Geometry::kCircular) return;
  cache_.last_lx = ctx.lx; cache_.last_ly = ctx.ly; cache_.last_n_modes = ctx.n_modes;
  bool is_square = ::std::abs(ctx.lx - ctx.ly) < 1e-9;
  double D = (ctx.e * ::std::pow(ctx.h, 3)) / (12.0 * (1.0 - ::std::pow(ctx.nu, 2)));
  cache_.ns.clear(); cache_.ms.clear();
  for (int n = 1; n <= ctx.n_modes; ++n) for (int m = 1; m <= ctx.n_modes; ++m) { cache_.ns.push_back(n); cache_.ms.push_back(m); }
  int num_modes = static_cast<int>(cache_.ns.size());
  cache_.modes.clear(); cache_.f_nm.resize(num_modes); cache_.omega_nm.resize(num_modes);
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(resolution_, 0.0, 1.0);
  Eigen::VectorXd x_phys = t * ctx.lx; Eigen::VectorXd y_phys = t * ctx.ly;
  ::std::vector<Eigen::VectorXd> phi_x(ctx.n_modes + 1), phi_y(ctx.n_modes + 1);
  ::std::vector<double> kx(ctx.n_modes + 1), ky(ctx.n_modes + 1);
  for (int i = 1; i <= ctx.n_modes; ++i) { phi_x[i] = free_beam_mode(i, x_phys, ctx.lx); kx[i] = get_beam_root(i) / ctx.lx; phi_y[i] = free_beam_mode(i, y_phys, ctx.ly); ky[i] = get_beam_root(i) / ctx.ly; }
  for (int k = 0; k < num_modes; ++k) {
    int n = cache_.ns[k], m = cache_.ms[k];
    Eigen::MatrixXd t1 = phi_y[m] * phi_x[n].transpose();
    if (is_square) cache_.modes.push_back(t1 + static_cast<double>(ctx.sign) * (phi_y[n] * phi_x[m].transpose()));
    else cache_.modes.push_back(t1);
    double omega_k = ::std::sqrt(D / (ctx.rho * ctx.h)) * (::std::pow(kx[n], 2) + ::std::pow(ky[m], 2));
    cache_.f_nm[k] = omega_k / (2.0 * M_PI); cache_.omega_nm[k] = omega_k;
  }
}

void PhysicsEngine::build_cache_circ(const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  if (cache_.last_lx == ctx.lx && cache_.last_n_modes == ctx.n_modes && ctx.geometry == Geometry::kCircular) return;
  cache_.last_lx = ctx.lx; cache_.last_n_modes = ctx.n_modes;
  double R = ctx.lx / 2.0; double D = (ctx.e * ::std::pow(ctx.h, 3)) / (12.0 * (1.0 - ::std::pow(ctx.nu, 2)));
  static const ::std::map<::std::pair<int, int>, double> bessel_zeros = { {{0, 1}, 2.4048}, {{0, 2}, 5.5201}, {{0, 3}, 8.6537}, {{1, 1}, 3.8317}, {{1, 2}, 7.0156}, {{1, 3}, 10.1735}, {{2, 1}, 5.1356}, {{2, 2}, 8.4172}, {{2, 3}, 11.6198}, {{3, 1}, 6.3802}, {{3, 2}, 9.7610}, {{3, 3}, 13.0152} };
  cache_.ns.clear(); cache_.ms.clear(); cache_.modes.clear();
  for (int n = 0; n < 4; ++n) for (int m = 1; m <= 3; ++m) {
      cache_.ns.push_back(n); cache_.ms.push_back(m);
      double lam = bessel_zeros.at({n, m}); double omega = (::std::pow(lam, 2) / ::std::pow(R, 2)) * ::std::sqrt(D / (ctx.rho * ctx.h));
      int k = static_cast<int>(cache_.ns.size()) - 1;
      cache_.f_nm.conservativeResize(k + 1); cache_.omega_nm.conservativeResize(k + 1);
      cache_.f_nm[k] = omega / (2.0 * M_PI); cache_.omega_nm[k] = omega;
      Eigen::MatrixXd mode(resolution_, resolution_);
      for (int i = 0; i < resolution_; ++i) for (int j = 0; j < resolution_; ++j) {
          double x = x_norm_(i, j) * ctx.lx, y = y_norm_(i, j) * ctx.lx, r_v = ::std::sqrt(x*x + y*y), th_v = ::std::atan2(y, x);
          mode(i, j) = (r_v <= R) ? ::std::cos(static_cast<double>(n) * th_v) * ::std::cyl_bessel_j(n, lam * r_v / R) : 0.0;
      }
      cache_.modes.push_back(mode);
  }
}

Eigen::MatrixXcd PhysicsEngine::compute_driven_response(double frequency, const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  if (ctx.geometry == Geometry::kCircular) build_cache_circ(ctx); else build_cache_rect(ctx);
  Eigen::VectorXcd F_nm = Eigen::VectorXcd::Zero(cache_.ns.size());
  double global_gain = speaker_drive_gain(frequency, ctx, true);
  for (const auto& t : ctx.transducers) {
    Eigen::VectorXd c_nm(cache_.ns.size());
    for (size_t k = 0; k < cache_.ns.size(); ++k) {
      int n = cache_.ns[k], m = cache_.ms[k]; double val = 0.0;
      if (ctx.geometry == Geometry::kCircular) {
        double R = ctx.lx / 2.0, r_t = ::std::sqrt(t.x*t.x + t.y*t.y), th_t = ::std::atan2(t.y, t.x);
        static const ::std::map<::std::pair<int, int>, double> bzeros = { {{0, 1}, 2.4048}, {{0, 2}, 5.5201}, {{0, 3}, 8.6537}, {{1, 1}, 3.8317}, {{1, 2}, 7.0156}, {{1, 3}, 10.1735}, {{2, 1}, 5.1356}, {{2, 2}, 8.4172}, {{2, 3}, 11.6198}, {{3, 1}, 6.3802}, {{3, 2}, 9.7610}, {{3, 3}, 13.0152} };
        if (bzeros.count({n, m})) { double lam = bzeros.at({n, m}); if (r_t <= R) val = ::std::cos(static_cast<double>(n) * th_t) * ::std::cyl_bessel_j(n, lam * r_t / R); }
      } else val = transducer_coupling_single(t.x, t.y, n, m, ctx.lx, ctx.ly, ctx.sign);
      c_nm[k] = val;
    }
    F_nm += (t.amplitude * ::std::complex<double>(::std::cos(t.phase_rad), ::std::sin(t.phase_rad))) * c_nm.cast<::std::complex<double>>();
  }
  F_nm *= global_gain;
  double omega = 2.0 * M_PI * frequency; Eigen::MatrixXcd resp = Eigen::MatrixXcd::Zero(resolution_, resolution_);
  for (size_t k = 0; k < cache_.ns.size(); ++k) {
    ::std::complex<double> denom = ::std::pow(cache_.omega_nm[k], 2) - ::std::pow(omega, 2) + ::std::complex<double>(0, 2.0 * ctx.damping * cache_.omega_nm[k] * omega);
    if (::std::abs(denom) > 1e-18) resp += (F_nm[k] / denom) * cache_.modes[k].cast<::std::complex<double>>();
  }
  return resp;
}

void PhysicsEngine::compute_visuals(double freq, const SimulationContext& ctx, Eigen::MatrixXd& sand, Eigen::MatrixXd& deformation) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  Eigen::MatrixXcd resp = compute_driven_response(freq, ctx);
  Eigen::MatrixXd amp = resp.array().abs(); double m_amp = amp.maxCoeff();
  if (m_amp > 1e-18) amp /= m_amp;
  sand = (-35.0 * amp.array().pow(2)).exp(); deformation = resp.real();
  double m_def = deformation.array().abs().maxCoeff(); if (m_def > 1e-18) deformation /= m_def;
}

bool PhysicsEngine::check_power_feasibility(double frequency, const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  if (ctx.geometry == Geometry::kCircular) build_cache_circ(ctx); else build_cache_rect(ctx);
  double true_gain = speaker_drive_gain(frequency, ctx, false);
  Eigen::VectorXcd F_nm_true = Eigen::VectorXcd::Zero(cache_.ns.size());
  double motor_efficiency = 3.5;
  for (const auto& t : ctx.transducers) {
      Eigen::VectorXd c_nm(cache_.ns.size());
      for (size_t k = 0; k < cache_.ns.size(); ++k) {
          int n = cache_.ns[k], m = cache_.ms[k]; double val = 0.0;
          if (ctx.geometry == Geometry::kCircular) {
              double R = ctx.lx / 2.0, r_t = ::std::sqrt(t.x*t.x + t.y*t.y), th_t = ::std::atan2(t.y, t.x);
              static const ::std::map<::std::pair<int, int>, double> bz = { {{0, 1}, 2.4048}, {{0, 2}, 5.5201}, {{0, 3}, 8.6537}, {{1, 1}, 3.8317}, {{1, 2}, 7.0156}, {{1, 3}, 10.1735}, {{2, 1}, 5.1356}, {{2, 2}, 8.4172}, {{2, 3}, 11.6198}, {{3, 1}, 6.3802}, {{3, 2}, 9.7610}, {{3, 3}, 13.0152} };
              if (bz.count({n, m})) { double lam = bz.at({n, m}); if (r_t <= R) val = ::std::cos(static_cast<double>(n) * th_t) * ::std::cyl_bessel_j(n, lam * r_t / R); }
          } else val = transducer_coupling_single(t.x, t.y, n, m, ctx.lx, ctx.ly, ctx.sign);
          c_nm[k] = val;
      }
      F_nm_true += (::std::sqrt(t.amplitude) * motor_efficiency * ::std::complex<double>(::std::cos(t.phase_rad), ::std::sin(t.phase_rad))) * c_nm.cast<::std::complex<double>>();
  }
  F_nm_true *= true_gain;
  double omega = 2.0 * M_PI * frequency; Eigen::VectorXcd coeffs = Eigen::VectorXcd::Zero(cache_.ns.size());
  for (size_t k = 0; k < cache_.ns.size(); ++k) {
      ::std::complex<double> denom = ::std::pow(cache_.omega_nm[k], 2) - ::std::pow(omega, 2) + ::std::complex<double>(0, 2.0 * ctx.damping * cache_.omega_nm[k] * omega);
      if (::std::abs(denom) > 1e-30) coeffs[k] = F_nm_true[k] / denom;
  }
  double max_disp = 0.0;
  for (size_t k = 0; k < cache_.ns.size(); ++k) {
      double peak = ::std::abs(coeffs[k]) * cache_.modes[k].array().abs().maxCoeff();
      if (peak > max_disp) max_disp = peak;
  }
  double max_accel = ::std::pow(omega, 2) * max_disp;
  return max_accel >= 9.81;
}

bool PhysicsEngine::is_degenerate(int n, int m, const SimulationContext& ctx) {
    if (ctx.geometry == Geometry::kRectangular) return false;
    double f1 = calculate_mode_frequency(n, m, ctx);
    double f2 = calculate_mode_frequency(m, n, ctx);
    return ::std::abs(f1 - f2) <= 1.0;
}

::std::vector<double> PhysicsEngine::snipe_phases(int n, int m, const SimulationContext& ctx) {
    ::std::vector<double> phases;
    bool degenerate = is_degenerate(n, m, ctx);
    for (size_t i = 0; i < ctx.transducers.size(); ++i) {
        if (degenerate) {
            static const double quadrature[] = {0.0, M_PI/2.0, M_PI, 3.0*M_PI/2.0};
            phases.push_back(quadrature[i % 4]);
        } else {
            double val = transducer_coupling_single(ctx.transducers[i].x, ctx.transducers[i].y, n, m, ctx.lx, ctx.ly, ctx.sign);
            phases.push_back(val >= 0 ? 0.0 : M_PI);
        }
    }
    return phases;
}

bool PhysicsEngine::validate_power(double frequency, const SimulationContext& ctx) {
    ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
    return check_power_feasibility(frequency, ctx);
}

void PhysicsEngine::clamp_transducer(Transducer& t, const SimulationContext& ctx) {
    double margin = 0.03;
    if (ctx.geometry == Geometry::kCircular) {
        double R = ctx.lx / 2.0 - margin;
        double r = ::std::sqrt(t.x*t.x + t.y*t.y);
        if (r > R && r > 1e-9) { t.x *= R/r; t.y *= R/r; }
    } else {
        double xm = ctx.lx / 2.0 - margin, ym = ctx.ly / 2.0 - margin;
        t.x = ::std::clamp(t.x, -xm, xm); t.y = ::std::clamp(t.y, -ym, ym);
    }
}

::std::vector<double> PhysicsEngine::calculate_spectrum(double f_min, double f_max, int n_points, const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  ::std::vector<double> spec(n_points); Eigen::VectorXd freqs = Eigen::VectorXd::LinSpaced(n_points, f_min, f_max);
  if (ctx.geometry == Geometry::kCircular) build_cache_circ(ctx); else build_cache_rect(ctx);
  for (int i = 0; i < n_points; ++i) {
    double f = freqs[i], omega = 2.0 * M_PI * f, energy = 0.0;
    for (size_t k = 0; k < cache_.ns.size(); ++k) {
        double d_sq = ::std::pow(::std::pow(cache_.omega_nm[k], 2) - ::std::pow(omega, 2), 2) + ::std::pow(2.0 * ctx.damping * cache_.omega_nm[k] * omega, 2);
        if (d_sq > 1e-18) energy += 1.0 / d_sq;
    }
    spec[i] = 10.0 * ::std::log10(energy + 1e-15);
  }
  return spec;
}

::std::vector<double> PhysicsEngine::get_resonant_frequencies(const SimulationContext& ctx) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_);
  if (ctx.geometry == Geometry::kCircular) build_cache_circ(ctx); else build_cache_rect(ctx);
  ::std::vector<double> res_f;
  for (size_t k = 0; k < cache_.ns.size(); ++k) {
      int n = cache_.ns[k], m = cache_.ms[k]; ::std::complex<double> F_k = 0;
      for (const auto& t : ctx.transducers) {
          double val = 0;
          if (ctx.geometry == Geometry::kCircular) {
              double R = ctx.lx / 2.0, r_t = ::std::sqrt(t.x*t.x + t.y*t.y), th_t = ::std::atan2(t.y, t.x);
              static const ::std::map<::std::pair<int, int>, double> bz = { {{0, 1}, 2.4048}, {{0, 2}, 5.5201}, {{0, 3}, 8.6537}, {{1, 1}, 3.8317}, {{1, 2}, 7.0156}, {{1, 3}, 10.1735}, {{2, 1}, 5.1356}, {{2, 2}, 8.4172}, {{2, 3}, 11.6198}, {{3, 1}, 6.3802}, {{3, 2}, 9.7610}, {{3, 3}, 13.0152} };
              if (bz.count({n, m})) { double lam = bz.at({n, m}); if (r_t <= R) val = ::std::cos(static_cast<double>(n) * th_t) * ::std::cyl_bessel_j(n, lam * r_t / R); }
          } else val = transducer_coupling_single(t.x, t.y, n, m, ctx.lx, ctx.ly, ctx.sign);
          F_k += t.amplitude * ::std::complex<double>(::std::cos(t.phase_rad), ::std::sin(t.phase_rad)) * val;
      }
      if (::std::abs(F_k) > 0.01) res_f.push_back(::std::round(cache_.f_nm[k] * 10.0) / 10.0);
  }
  ::std::sort(res_f.begin(), res_f.end()); res_f.erase(::std::unique(res_f.begin(), res_f.end()), res_f.end());
  return res_f;
}

double PhysicsEngine::transducer_coupling_single(double tx, double ty, int n, int m, double lx, double ly, int sign) {
  auto get_root = [](int i) {
      static const ::std::vector<double> roots = { 4.730040744862704, 7.853204624095838, 10.995607838001671, 14.137165491257464, 17.278759657399482, 20.420352251041250, 23.561944901923447, 26.703537555510351, 29.845130209103030, 32.986722862692830, 36.128315516282620 };
      if (i >= 1 && i <= (int)roots.size()) return roots[i - 1];
      return (static_cast<double>(i) + 0.5) * M_PI;
  };
  auto fval = [&](int i, double x, double L) {
      double beta = get_root(i), k = beta / ::std::max(L, 1e-12), sigma = (::std::cosh(beta) - ::std::cos(beta)) / (::std::sinh(beta) - ::std::sin(beta));
      return ::std::cosh(k * x) + ::std::cos(k * x) - sigma * (::std::sinh(k * x) + ::std::sin(k * x));
  };
  double tx_s = tx + lx / 2.0, ty_s = ty + ly / 2.0, val = fval(n, tx_s, lx) * fval(m, ty_s, ly);
  if (::std::abs(lx - ly) < 1e-9) val += static_cast<double>(sign) * fval(m, tx_s, lx) * fval(n, ty_s, ly);
  return val;
}

void PhysicsEngine::init_particles(int n, double lx, double ly) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_); particles_.resize(n, 2); particle_vel_ = Eigen::MatrixXd::Zero(n, 2);
  ::std::random_device rd; ::std::mt19937 gen(rd()); ::std::uniform_real_distribution<> dx(-lx/2.0, lx/2.0), dy(-ly/2.0, ly/2.0);
  for (int i = 0; i < n; ++i) { particles_(i, 0) = dx(gen); particles_(i, 1) = dy(gen); }
}

void PhysicsEngine::step_particles(const Eigen::MatrixXcd& resp, double lx, double ly, double dt) {
  ::std::lock_guard<::std::recursive_mutex> lock(mutex_); if (particles_.rows() == 0) return;
  Eigen::MatrixXd as = resp.array().abs2(); double mas = as.maxCoeff(); if (mas > 1e-18) as /= mas;
  Eigen::MatrixXd gx(resolution_, resolution_), gy(resolution_, resolution_);
  double dgx = lx / (resolution_ - 1), dgy = ly / (resolution_ - 1);
  for (int i = 0; i < resolution_; ++i) for (int j = 0; j < resolution_; ++j) {
      int pi = ::std::max(0, i-1), ni = ::std::min(resolution_-1, i+1), pj = ::std::max(0, j-1), nj = ::std::min(resolution_-1, j+1);
      gx(i, j) = (as(i, nj) - as(i, pj)) / (nj == pj ? 1.0 : (nj - pj) * dgx);
      gy(i, j) = (as(ni, j) - as(pi, j)) / (ni == pi ? 1.0 : (ni - pi) * dgy);
  }
  double fs = 0.5, fr = 0.85;
  for (int i = 0; i < particles_.rows(); ++i) {
    double px = particles_(i, 0), py = particles_(i, 1);
    int gxi = ::std::clamp(static_cast<int>((px/lx + 0.5)*(resolution_-1)), 0, resolution_-1), gyi = ::std::clamp(static_cast<int>((py/ly + 0.5)*(resolution_-1)), 0, resolution_-1);
    double fxc = -gx(gyi, gxi) * fs, fyc = -gy(gyi, gxi) * fs;
    particle_vel_(i, 0) = particle_vel_(i, 0) * fr + fxc * dt; particle_vel_(i, 1) = particle_vel_(i, 1) * fr + fyc * dt;
    particles_(i, 0) += particle_vel_(i, 0) * dt; particles_(i, 1) += particle_vel_(i, 1) * dt;
    if (::std::abs(particles_(i, 0)) > lx / 2.0) { particles_(i, 0) = (particles_(i, 0)>0?1.0:-1.0)*lx/2.0; particle_vel_(i, 0) *= -0.5; }
    if (::std::abs(particles_(i, 1)) > ly / 2.0) { particles_(i, 1) = (particles_(i, 1)>0?1.0:-1.0)*ly/2.0; particle_vel_(i, 1) *= -0.5; }
  }
}

} // namespace chladni
