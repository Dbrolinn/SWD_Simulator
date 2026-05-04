/**
 * @file analyzer.cpp
 * @brief Multi-threaded analyzer for plate resonance optimization.
 */

#include "analyzer.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace chladni {

Analyzer::Analyzer(::std::shared_ptr<PhysicsEngine> physics) : physics_(physics) {}

void Analyzer::repair_layout(::std::vector<Transducer>& layout, const SimulationContext& ctx) {
    for (auto& t : layout) {
        physics_->clamp_transducer(t, ctx);
    }

    bool violation = true;
    int max_iterations = 10;
    while (violation && max_iterations--) {
        violation = false;
        for (size_t i = 0; i < layout.size(); ++i) {
            for (size_t j = i + 1; j < layout.size(); ++j) {
                double dx = layout[i].x - layout[j].x;
                double dy = layout[i].y - layout[j].y;
                double dist = ::std::sqrt(dx * dx + dy * dy);
                if (dist < kMinClearanceMeters) {
                    violation = true;
                    double overlap = kMinClearanceMeters - dist;
                    double nx = (dist < 1e-6) ? 1.0 : dx / dist;
                    double ny = (dist < 1e-6) ? 0.0 : dy / dist;
                    layout[i].x += nx * overlap * 0.5;
                    layout[i].y += ny * overlap * 0.5;
                    layout[j].x -= nx * overlap * 0.5;
                    layout[j].y -= ny * overlap * 0.5;
                }
            }
        }
        for (auto& t : layout) physics_->clamp_transducer(t, ctx);
    }
}

::std::vector<Transducer> Analyzer::crossover(const ::std::vector<Transducer>& p1, const ::std::vector<Transducer>& p2) {
    ::std::vector<Transducer> child;
    ::std::random_device rd; ::std::mt19937 gen(rd());
    ::std::uniform_int_distribution<> coin(0, 1);
    for (size_t i = 0; i < p1.size(); ++i) {
        child.push_back(coin(gen) ? p1[i] : p2[i]);
    }
    return child;
}

void Analyzer::mutate(::std::vector<Transducer>& layout, double rate, const SimulationContext& ctx) {
    ::std::random_device rd; ::std::mt19937 gen(rd());
    ::std::uniform_real_distribution<> prob(0.0, 1.0);
    ::std::normal_distribution<> offset(0.0, 0.02);
    for (auto& t : layout) {
        if (prob(gen) < rate) {
            t.x += offset(gen);
            t.y += offset(gen);
        }
    }
    repair_layout(layout, ctx);
}

LayoutResult Analyzer::evaluate_layout(const ::std::vector<Transducer>& layout, const SimulationContext& base_ctx) {
    LayoutResult res;
    res.best_layout = layout;
    res.alphabet_size = 0;
    res.feasibility_rate = 0.0;
    res.total_displacement = 0.0;

    SimulationContext eval_ctx = base_ctx;
    eval_ctx.transducers = layout;

    struct ModeInfo {
        double freq;
        Eigen::MatrixXd energy;
        double total_disp;
    };
    ::std::vector<ModeInfo> unique_modes;
    int feasible_count = 0;
    int total_evaluated = 0;

    for (int n = 1; n <= 10; ++n) {
        for (int m = 1; m <= 10; ++m) {
            total_evaluated++;
            double freq = physics_->calculate_mode_frequency(n, m, eval_ctx);
            ::std::vector<double> phases = physics_->snipe_phases(n, m, eval_ctx);
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].phase_rad = phases[i];
                eval_ctx.transducers[i].frequency = freq;
            }

            // Binary search for minimum digital amplitude (0.1 to 1.0) required for 10G
            if (!physics_->validate_power(freq, eval_ctx)) continue;
            
            feasible_count++;
            double low = 0.1, high = 1.0, best_amp = 1.0;
            
            // Hard clamp at 0.1 if it already validates at 0.1
            for (auto& t : eval_ctx.transducers) t.amplitude = 0.1;
            if (physics_->validate_power(freq, eval_ctx)) {
                best_amp = 0.1;
            } else {
                for (int iter = 0; iter < 10; ++iter) {
                    double mid = (low + high) / 2.0;
                    for (auto& t : eval_ctx.transducers) t.amplitude = mid;
                    if (physics_->validate_power(freq, eval_ctx)) {
                        high = mid;
                        best_amp = mid;
                    } else {
                        low = mid;
                    }
                }
            }
            for (auto& t : eval_ctx.transducers) t.amplitude = best_amp;

            Eigen::MatrixXcd resp = physics_->compute_driven_response(freq, eval_ctx);
            Eigen::MatrixXd energy = resp.array().abs();
            double total_disp = energy.sum();

            bool unique = true;
            for (auto it = unique_modes.begin(); it != unique_modes.end(); ) {
                // Rule 2: Frequency Debounce (10.0 Hz)
                if (::std::abs(freq - it->freq) < 10.0) {
                    // Rule 1: Absolute Energy Similarity (> 90%)
                    double dot = (energy.array() * it->energy.array()).sum();
                    double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                    double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;

                    if (similarity > 0.90) {
                        if (total_disp > it->total_disp) {
                            it = unique_modes.erase(it);
                            continue;
                        } else {
                            unique = false;
                            break;
                        }
                    }
                }
                
                // Also check similarity for non-proximal frequencies
                double dot = (energy.array() * it->energy.array()).sum();
                double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;
                if (similarity > 0.98) {
                    unique = false;
                    break;
                }
                ++it;
            }

            if (unique) {
                unique_modes.push_back({freq, energy, total_disp});
            }
        }
    }
    res.alphabet_size = static_cast<int>(unique_modes.size());
    res.feasibility_rate = (total_evaluated > 0) ? (double)feasible_count / total_evaluated : 0.0;
    
    for (const auto& m : unique_modes) res.total_displacement += m.total_disp;
    
    return res;
}

::std::vector<LayoutResult> Analyzer::run_symmetric_grid_search(const SimulationContext& base_ctx, const GridParams& params) {
    grid_progress_ = 0.0f;
    grid_best_alphabet_ = 0;
    ::std::cout << "[GridExplorer] Starting Symmetric Search..." << ::std::endl;

    ::std::vector<LayoutResult> candidates;
    
    // Plate dimensions
    double lx = base_ctx.lx;
    double ly = (base_ctx.geometry == Geometry::kCircular) ? base_ctx.lx : base_ctx.ly;

    // Search range with edge gap
    // dx and dy are distances from center, so max is (L/2 - gap)
    // Minimum offset should also respect the radius if we don't want them crossing the origin
    double dx_start = 0.025, dx_end = lx / 2.0 - params.edge_gap_m;
    double dy_start = 0.025, dy_end = ly / 2.0 - params.edge_gap_m;
    double dx_step = params.step_size_m;
    double dy_step = params.step_size_m;

    if (dx_end < dx_start || dy_end < dy_start) {
        ::std::cerr << "[GridExplorer] Error: Plate too small or edge gap too large." << ::std::endl;
        return candidates;
    }
    
    int nx = static_cast<int>((dx_end - dx_start)/dx_step + 1);
    int ny = static_cast<int>((dy_end - dy_start)/dy_step + 1);
    int total_steps = nx * ny;
    int current_step = 0;

    for (double dx = dx_start; dx <= dx_end + 1e-6; dx += dx_step) {
        for (double dy = dy_start; dy <= dy_end + 1e-6; dy += dy_step) {
            grid_progress_ = (float)current_step++ / total_steps;

            ::std::vector<Transducer> layout = {
                {dx, dy, 1.0, 0.0, ::std::nullopt},
                {-dx, dy, 1.0, 0.0, ::std::nullopt},
                {-dx, -dy, 1.0, 0.0, ::std::nullopt},
                {dx, -dy, 1.0, 0.0, ::std::nullopt}
            };

            // Hardware clearance check (25mm radius + 5mm gap + 25mm radius = 55mm)
            bool valid = true;
            for (size_t i = 0; i < layout.size(); ++i) {
                for (size_t j = i + 1; j < layout.size(); ++j) {
                    double dist = ::std::sqrt(::std::pow(layout[i].x - layout[j].x, 2) + ::std::pow(layout[i].y - layout[j].y, 2));
                    if (dist < 0.055) { valid = false; break; }
                }
                if (!valid) break;
            }

            if (valid) {
                LayoutResult res = evaluate_layout(layout, base_ctx);
                if (res.alphabet_size > grid_best_alphabet_) {
                    grid_best_alphabet_ = res.alphabet_size;
                    ::std::cout << "[GridExplorer] New Best Alphabet: " << grid_best_alphabet_.load() << " at dx=" << dx << ", dy=" << dy << ::std::endl;
                }
                candidates.push_back(res);
            }
        }
    }

    ::std::sort(candidates.begin(), candidates.end(), [](const LayoutResult& a, const LayoutResult& b) {
        if (a.alphabet_size != b.alphabet_size) return a.alphabet_size > b.alphabet_size;
        return a.total_displacement > b.total_displacement;
    });

    if (candidates.size() > 10) candidates.resize(10);
    grid_progress_ = 1.0f;
    return candidates;
}

void Analyzer::export_to_json(const ::std::string& path, const ::std::vector<LayoutResult>& results, const SimulationContext& ctx) {
    auto round_to = [](double val, int decimals) {
        double p = ::std::pow(10, decimals);
        return ::std::round(val * p) / p;
    };

    nlohmann::json root = nlohmann::json::array();
    int current_id = 1;
    for (const auto& res : results) {
        nlohmann::json symbol;
        double freq = res.best_layout.empty() ? 0.0 : res.best_layout[0].frequency.value_or(0.0);
        int freq_int = static_cast<int>(::std::round(freq));
        
        symbol["id"] = current_id++;
        symbol["display_name"] = "CHLADNI_" + ::std::to_string(freq_int);
        
        nlohmann::json hw_config;
        hw_config["base_volume_1"] = round_to(ctx.base_volume_1, 3);
        hw_config["base_volume_2"] = round_to(ctx.base_volume_2, 3);
        
        nlohmann::json channels = nlohmann::json::array();
        for (size_t i = 0; i < res.best_layout.size(); ++i) {
            const auto& t = res.best_layout[i];
            nlohmann::json entry;
            entry["channel"] = static_cast<int>(i + 1);
            entry["x"] = round_to(t.x, 3);
            entry["y"] = round_to(t.y, 3);
            entry["frequency_hz"] = round_to(t.frequency.value_or(0.0), 1);
            
            double phase_deg = t.phase_rad * 180.0 / M_PI;
            // Round to nearest 0, 90, 180, 270
            int rounded_phase = static_cast<int>(::std::round(phase_deg / 90.0) * 90.0) % 360;
            if (rounded_phase < 0) rounded_phase += 360;
            
            entry["phase_deg"] = rounded_phase;
            entry["amplitude"] = round_to(t.amplitude, 3);
            channels.push_back(entry);
        }
        hw_config["channels"] = channels;
        symbol["hardware_config"] = hw_config;
        
        nlohmann::json ui_metadata;
        ui_metadata["image_path"] = "./dictionary/CHLADNI_" + ::std::to_string(freq_int) + ".png";
        symbol["ui_metadata"] = ui_metadata;
        
        nlohmann::json physical_params;
        physical_params["settling_time_ms"] = nullptr;
        symbol["physical_parameters"] = physical_params;
        
        root.push_back(symbol);
    }
    ::std::ofstream file(path);
    if (file.is_open()) {
        file << root.dump(4);
    }
}

::std::vector<LayoutResult> Analyzer::run_sensitivity_sweep(const SimulationContext& base_ctx) {
    sweep_progress_ = 0.0f;
    ::std::vector<LayoutResult> results;
    SimulationContext eval_ctx = base_ctx;

    struct ModeInfo {
        double freq;
        Eigen::MatrixXd energy;
        double total_disp;
        ::std::vector<Transducer> layout;
        ::std::string type;
    };
    ::std::vector<ModeInfo> unique_modes;

    int total_modes = 100;
    int current_mode = 0;
    for (int n = 1; n <= 10; ++n) {
        for (int m = 1; m <= 10; ++m) {
            sweep_progress_ = (float)current_mode++ / total_modes;
            double theoretical_f = physics_->calculate_mode_frequency(n, m, eval_ctx);
            
            double best_f = theoretical_f;
            double max_peak = 0.0;
            
            // Sweep around theoretical +/- 5% to find peak resonance
            for (int i = 0; i <= 20; ++i) {
                double f = theoretical_f * (0.95 + 0.1 * i / 20.0);
                Eigen::MatrixXcd resp = physics_->compute_driven_response(f, eval_ctx);
                double peak = resp.array().abs().maxCoeff();
                if (peak > max_peak) {
                    max_peak = peak;
                    best_f = f;
                }
            }
            
            double freq = best_f;
            ::std::vector<double> phases = physics_->snipe_phases(n, m, eval_ctx);
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].phase_rad = phases[i];
                eval_ctx.transducers[i].frequency = freq;
                eval_ctx.transducers[i].amplitude = 1.0;
            }

            if (!physics_->validate_power(freq, eval_ctx)) continue;

            // Binary search for minimum digital amplitude (0.1 to 1.0)
            double low = 0.1, high = 1.0, best_amp = 1.0;
            for (auto& t : eval_ctx.transducers) t.amplitude = 0.1;
            if (physics_->validate_power(freq, eval_ctx)) {
                best_amp = 0.1;
            } else {
                for (int iter = 0; iter < 10; ++iter) {
                    double mid = (low + high) / 2.0;
                    for (auto& t : eval_ctx.transducers) t.amplitude = mid;
                    if (physics_->validate_power(freq, eval_ctx)) {
                        high = mid;
                        best_amp = mid;
                    } else {
                        low = mid;
                    }
                }
            }
            for (auto& t : eval_ctx.transducers) t.amplitude = best_amp;

            Eigen::MatrixXcd resp = physics_->compute_driven_response(freq, eval_ctx);
            Eigen::MatrixXd energy = resp.array().abs();
            double total_disp = energy.sum();

            bool unique = true;
            for (auto it = unique_modes.begin(); it != unique_modes.end(); ) {
                if (::std::abs(freq - it->freq) < 10.0) {
                    double dot = (energy.array() * it->energy.array()).sum();
                    double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                    double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;

                    if (similarity > 0.90) {
                        if (total_disp > it->total_disp) {
                            it = unique_modes.erase(it);
                            continue;
                        } else {
                            unique = false;
                            break;
                        }
                    }
                }
                double dot = (energy.array() * it->energy.array()).sum();
                double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;
                if (similarity > 0.98) {
                    unique = false;
                    break;
                }
                ++it;
            }

            if (unique) {
                unique_modes.push_back({freq, energy, total_disp, eval_ctx.transducers, "Mode_" + ::std::to_string(n) + "_" + ::std::to_string(m)});
            }
        }
    }
    
    for (const auto& m : unique_modes) {
        LayoutResult res;
        res.best_layout = m.layout;
        res.alphabet_size = 1;
        res.layout_type = m.type;
        res.total_displacement = m.total_disp;
        results.push_back(res);
    }

    ::std::sort(results.begin(), results.end(), [](const LayoutResult& a, const LayoutResult& b) {
        return a.best_layout[0].frequency.value_or(0.0) < b.best_layout[0].frequency.value_or(0.0);
    });

    sweep_progress_ = 1.0f;
    return results;
}

::std::vector<LayoutResult> Analyzer::run_grid_search(const SimulationContext& base_ctx) {
    ::std::vector<LayoutResult> results;
    (void)base_ctx;
    return results;
}

} // namespace chladni
