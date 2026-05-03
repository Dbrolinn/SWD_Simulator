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

    SimulationContext eval_ctx = base_ctx;
    eval_ctx.transducers = layout;

    ::std::vector<Eigen::VectorXcd> unique_signatures;
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

            // Binary search for minimum amplitude (0.0 to 25.0 Watts) required for 1G
            double low = 0.0, high = 25.0, best_amp = 25.0;
            if (!physics_->validate_power(freq, eval_ctx)) continue;
            
            feasible_count++;

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
            for (auto& t : eval_ctx.transducers) t.amplitude = best_amp;

            Eigen::MatrixXcd resp = physics_->compute_driven_response(freq, eval_ctx);
            Eigen::VectorXcd sig = Eigen::Map<Eigen::VectorXcd>(resp.data(), resp.size());
            double norm = sig.norm();
            if (norm < 1e-12) continue;
            sig /= norm;

            bool unique = true;
            for (const auto& existing : unique_signatures) {
                if (::std::abs(sig.dot(existing)) > 0.98) { unique = false; break; }
            }
            if (unique) {
                unique_signatures.push_back(sig);
                res.alphabet_size++;
            }
        }
    }
    res.feasibility_rate = (total_evaluated > 0) ? (double)feasible_count / total_evaluated : 0.0;
    return res;
}

::std::vector<LayoutResult> Analyzer::run_genetic_algorithm(const SimulationContext& base_ctx, const GAParams& params) {
    ::std::vector<LayoutResult> population;
    ::std::random_device rd; ::std::mt19937 gen(rd());
    ::std::uniform_real_distribution<> dis_x(-base_ctx.lx/2, base_ctx.lx/2);
    ::std::uniform_real_distribution<> dis_y(-base_ctx.ly/2, base_ctx.ly/2);

    for (int i = 0; i < params.population_size; ++i) {
        ::std::vector<Transducer> layout;
        for (int j = 0; j < params.transducer_count; ++j) {
            layout.push_back({dis_x(gen), dis_y(gen), 25.0, 0.0, ::std::nullopt});
        }
        repair_layout(layout, base_ctx);
        population.push_back({0, 0.0, 0.0, layout, "GA_Initial", 0, 0});
    }

    for (int gen_idx = 0; gen_idx < params.generations; ++gen_idx) {
        ::std::vector<::std::future<LayoutResult>> futures;
        for (const auto& individual : population) {
            futures.push_back(::std::async(::std::launch::async, [this, individual, base_ctx]() {
                return evaluate_layout(individual.best_layout, base_ctx);
            }));
        }
        population.clear();
        for (auto& f : futures) population.push_back(f.get());

        ::std::sort(population.begin(), population.end(), [](const LayoutResult& a, const LayoutResult& b) {
            return a.alphabet_size > b.alphabet_size;
        });

        if (gen_idx == params.generations - 1) break;

        ::std::vector<LayoutResult> next_gen;
        int elite_count = ::std::max(1, (int)(0.1 * params.population_size));
        for (int i = 0; i < elite_count; ++i) next_gen.push_back(population[i]);

        while (next_gen.size() < (size_t)params.population_size) {
            auto select_parent = [&]() {
                ::std::uniform_int_distribution<> dist(0, (int)population.size() - 1);
                int b1 = dist(gen), b2 = dist(gen), b3 = dist(gen);
                int best = b1;
                if (population[b2].alphabet_size > population[best].alphabet_size) best = b2;
                if (population[b3].alphabet_size > population[best].alphabet_size) best = b3;
                return population[best].best_layout;
            };
            ::std::vector<Transducer> child = crossover(select_parent(), select_parent());
            mutate(child, params.mutation_rate, base_ctx);
            next_gen.push_back({0, 0.0, 0.0, child, "GA_Evolved", 0, 0});
        }
        population = next_gen;
    }

    export_to_json(params.export_path, population);
    return population;
}

void Analyzer::export_to_json(const ::std::string& path, const ::std::vector<LayoutResult>& results) {
    nlohmann::json root = nlohmann::json::array();
    for (const auto& res : results) {
        nlohmann::json symbol = nlohmann::json::array();
        for (size_t i = 0; i < res.best_layout.size(); ++i) {
            const auto& t = res.best_layout[i];
            nlohmann::json entry;
            entry["base_volume"] = 1.0;
            entry["channel"] = static_cast<int>(i);
            entry["frequency_hz"] = t.frequency.value_or(0.0);
            entry["phase_deg"] = t.phase_rad * 180.0 / M_PI;
            entry["amplitude"] = t.amplitude;
            entry["x"] = t.x;
            entry["y"] = t.y;
            symbol.push_back(entry);
        }
        root.push_back(symbol);
    }
    ::std::ofstream file(path);
    if (file.is_open()) {
        file << root.dump(4);
    }
}

::std::vector<LayoutResult> Analyzer::run_sensitivity_sweep(const SimulationContext& base_ctx) {
    ::std::vector<LayoutResult> results;
    SimulationContext eval_ctx = base_ctx;

    for (int n = 1; n <= 10; ++n) {
        for (int m = 1; m <= 10; ++m) {
            double theoretical_f = physics_->calculate_mode_frequency(n, m, eval_ctx);
            
            double best_f = theoretical_f;
            double max_val = 0.0;
            
            // Sweep around theoretical +/- 5% to find peak resonance
            for (int i = 0; i <= 20; ++i) {
                double f = theoretical_f * (0.95 + 0.1 * i / 20.0);
                Eigen::MatrixXcd resp = physics_->compute_driven_response(f, eval_ctx);
                double peak = resp.array().abs().maxCoeff();
                if (peak > max_val) {
                    max_val = peak;
                    best_f = f;
                }
            }
            
            double freq = best_f;
            ::std::vector<double> phases = physics_->snipe_phases(n, m, eval_ctx);
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].phase_rad = phases[i];
                eval_ctx.transducers[i].frequency = freq;
                eval_ctx.transducers[i].amplitude = 25.0; // Start at max for validation
            }

            if (!physics_->validate_power(freq, eval_ctx)) continue;

            // Binary search for minimum amplitude
            double low = 0.0, high = 25.0, best_amp = 25.0;
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
            for (auto& t : eval_ctx.transducers) t.amplitude = best_amp;

            LayoutResult res;
            res.best_layout = eval_ctx.transducers;
            res.alphabet_size = 1;
            res.layout_type = "Mode_" + ::std::to_string(n) + "_" + ::std::to_string(m);
            results.push_back(res);
        }
    }
    
    ::std::sort(results.begin(), results.end(), [](const LayoutResult& a, const LayoutResult& b) {
        return a.best_layout[0].frequency.value_or(0.0) < b.best_layout[0].frequency.value_or(0.0);
    });

    return results;
}

::std::vector<LayoutResult> Analyzer::run_grid_search(const SimulationContext& base_ctx) {
    ::std::vector<LayoutResult> results;
    (void)base_ctx;
    return results;
}

} // namespace chladni
