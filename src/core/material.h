/**
 * @file material.h
 * @brief This file defines the material and transducer properties for plate resonance.
 */

#pragma once

#include <string>
#include <vector>

namespace chladni {

/**
 * @struct MaterialSpec
 * @brief Material preset for real metal sheets available on the market.
 */
struct MaterialSpec {
  ::std::string name;
  double e;           ///< Young's Modulus (Pa)
  double rho;         ///< Density (kg/m^3)
  double nu;          ///< Poisson's Ratio
  struct Size {
    int w_mm;
    int h_mm;
  };
  ::std::vector<Size> sizes_mm;
  ::std::vector<double> thicknesses_mm;
};

/**
 * @struct VibrationSpeaker
 * @brief Physical constraints for a real vibration speaker transducer.
 */
struct VibrationSpeaker {
  double f_min = 20.0;
  double f_max = 20000.0;
  double nominal_power_w = 25.0;
  double diameter_m = 0.05;
  double mass_kg = 0.268;
};

// ── Constants ─────────────────────────────────────────────────────────────

}  // namespace chladni
