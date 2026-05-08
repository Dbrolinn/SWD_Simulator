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
#include <thread>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace chladni {

Analyzer::Analyzer(::std::shared_ptr<PhysicsEngine> physics) : physics_(physics) {}

void Analyzer::repair_layout(::std::vector<Transducer>& layout, const SimulationContext& ctx) {
    for (auto& t : layout) { physics_->clamp_transducer(t, ctx); }
    float min_clearance = (2.0f * ctx.transducer_radius_m) + ctx.transducer_spacing_m;
    bool violation = true;
    int max_iterations = 10;
    while (violation && max_iterations--) {
        violation = false;
        for (size_t i = 0; i < layout.size(); ++i) {
            for (size_t j = i + 1; j < layout.size(); ++j) {
                float dx = layout[i].x - layout[j].x;
                float dy = layout[i].y - layout[j].y;
                float dist = ::std::sqrt(dx * dx + dy * dy);
                if (dist < min_clearance) {
                    violation = true;
                    float overlap = min_clearance - dist;
                    float nx = (dist < 1e-6f) ? 1.0f : dx / dist;
                    float ny = (dist < 1e-6f) ? 0.0f : dy / dist;
                    layout[i].x += nx * overlap * 0.5f;
                    layout[i].y += ny * overlap * 0.5f;
                    layout[j].x -= nx * overlap * 0.5f;
                    layout[j].y -= ny * overlap * 0.5f;
                }
            }
        }
        for (auto& t : layout) physics_->clamp_transducer(t, ctx);
    }
}

LayoutResult Analyzer::evaluate_layout(const ::std::vector<Transducer>& layout, const SimulationContext& base_ctx) {
    LayoutResult res;
    res.best_layout = layout;
    res.alphabet_size = 0;
    res.feasibility_rate = 0.0f;
    res.total_displacement = 0.0f;

    SimulationContext eval_ctx = base_ctx;
    eval_ctx.transducers = layout;

    struct ModeInfo { double freq; Eigen::MatrixXd energy; double total_disp; };
    ::std::vector<ModeInfo> unique_modes;
    int feasible_count = 0;
    int total_evaluated = 0;

    for (int n = 1; n <= eval_ctx.n_modes; ++n) {
        for (int m = 1; m <= eval_ctx.n_modes; ++m) {
            total_evaluated++;
            modes_evaluated_++; 
            
            double theoretical_f = physics_->calculate_mode_frequency(n, m, eval_ctx);
            double base_freq = (theoretical_f * base_ctx.calib_m) + base_ctx.calib_b;
            if (base_freq > base_ctx.max_frequency) continue;

            ::std::vector<double> phases = physics_->snipe_phases(n, m, eval_ctx);
            double max_c = 1e-9;
            ::std::vector<double> couplings(eval_ctx.transducers.size());
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                couplings[i] = ::std::abs(PhysicsEngine::transducer_coupling_single(
                    eval_ctx.transducers[i].x, eval_ctx.transducers[i].y, n, m, eval_ctx.lx, eval_ctx.ly, eval_ctx.sign));
                if (couplings[i] > max_c) max_c = couplings[i];
            }

            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].phase_rad = phases[i];
                eval_ctx.transducers[i].frequency = base_freq;
                eval_ctx.transducers[i].amplitude = couplings[i] / max_c;
            }

            feasible_count++;

            double best_f = base_freq;
            double max_peak = 0.0;
            Eigen::MatrixXcd best_resp;
            
            for (double f = base_freq - 10.0; f <= base_freq + 10.0; f += 10.0) {
                Eigen::MatrixXcd resp = physics_->compute_driven_response(f, eval_ctx);
                double peak = resp.array().abs().maxCoeff();
                if (peak > max_peak) { max_peak = peak; best_f = f; best_resp = std::move(resp); }
            }

            double freq = best_f;
            Eigen::MatrixXd energy = best_resp.array().abs();
            double total_disp = energy.sum();

            bool unique = true;
            for (auto it = unique_modes.begin(); it != unique_modes.end(); ) {
                if (::std::abs(freq - it->freq) < 30.0) {
                    double dot = (energy.array() * it->energy.array()).sum();
                    double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                    double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;
                    if (similarity > 0.85) { 
                        if (total_disp > it->total_disp) { it = unique_modes.erase(it); continue; } 
                        else { unique = false; break; }
                    }
                }
                double dot = (energy.array() * it->energy.array()).sum();
                double norm_prod = ::std::sqrt((energy.array().pow(2)).sum()) * ::std::sqrt((it->energy.array().pow(2)).sum());
                double similarity = (norm_prod > 1e-12) ? dot / norm_prod : 0.0;
                if (similarity > 0.95) { unique = false; break; }
                ++it;
            }
            if (unique) unique_modes.push_back({freq, energy, total_disp});
        }
    }
    res.alphabet_size = static_cast<int>(unique_modes.size());
    res.feasibility_rate = (total_evaluated > 0) ? (double)feasible_count / total_evaluated : 0.0;
    for (const auto& m : unique_modes) res.total_displacement += m.total_disp;
    return res;
}

::std::vector<LayoutResult> Analyzer::run_symmetric_grid_search(const SimulationContext& base_ctx, const GridParams& params) {
    is_analyzing_ = true;
    grid_progress_ = 0.0f;
    modes_evaluated_ = 0;
    grid_best_alphabet_ = 0;
    top_layouts_.clear();

    float lx = static_cast<float>(base_ctx.lx);
    float ly = static_cast<float>((base_ctx.geometry == Geometry::kCircular) ? base_ctx.lx : base_ctx.ly);
    float active_radius_m = static_cast<float>(base_ctx.transducer_radius_m);

    float dx_start = params.use_roi ? params.roi_dx_min : params.start_offset_m;
    float dy_start = params.use_roi ? params.roi_dy_min : params.start_offset_m;
    float dx_end = params.use_roi ? params.roi_dx_max : (lx / 2.0f - params.edge_gap_m - active_radius_m);
    float dy_end = params.use_roi ? params.roi_dy_max : (ly / 2.0f - params.edge_gap_m - active_radius_m);

    if (dx_end < dx_start) dx_end = dx_start;
    if (dy_end < dy_start) dy_end = dy_start;

    int nx = ::std::max(1, (int)::std::floor((dx_end - dx_start) / params.step_size_m) + 1);
    int ny = ::std::max(1, (int)::std::floor((dy_end - dy_start) / params.step_size_m) + 1);

    heatmap_.nx = nx;
    heatmap_.ny = ny;
    heatmap_.dx_min = dx_start;
    heatmap_.dx_max = dx_start + (nx - 1) * params.step_size_m;
    heatmap_.dy_min = dy_start;
    heatmap_.dy_max = dy_start + (ny - 1) * params.step_size_m;
    heatmap_.scores.assign(nx * ny, 0.0f);
    heatmap_.valid = true;

    auto generate_layouts = [&](double dx, double dy) -> ::std::vector<::std::pair<::std::string, ::std::vector<Transducer>>> {
        ::std::vector<::std::pair<::std::string, ::std::vector<Transducer>>> layouts;
        int count = base_ctx.transducers.size();
        
        if (count == 1) {
            layouts.push_back({"Single", {{static_cast<float>(dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}}});
        } else if (count == 2) {
            if (::std::abs(dy - dx) < 1e-6) {
                layouts.push_back({"Diagonal Pair", {{static_cast<float>(dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}}});
                layouts.push_back({"Anti-Diagonal Pair", {{static_cast<float>(dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}}});
            } else {
                if (::std::abs(dy - static_cast<double>(params.start_offset_m)) < 1e-6) layouts.push_back({"Horizontal Pair", {{static_cast<float>(dx), 0.0f, 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), 0.0f, 1.0f, 0.0f, ::std::nullopt}}});
                if (::std::abs(dx - static_cast<double>(params.start_offset_m)) < 1e-6) layouts.push_back({"Vertical Pair", {{0.0f, static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}, {0.0f, static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}}});
                layouts.push_back({"Diagonal Pair", {{static_cast<float>(dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}}});
                layouts.push_back({"Anti-Diagonal Pair", {{static_cast<float>(dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}}});
            }
        } else if (count == 3) {
            double r = ::std::sqrt(dx*dx + dy*dy);
            layouts.push_back({"Eq. Triangle", { {0.0f, static_cast<float>(r), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(r * ::std::cos(-M_PI/6.0)), static_cast<float>(r * ::std::sin(-M_PI/6.0)), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-r * ::std::cos(-M_PI/6.0)), static_cast<float>(r * ::std::sin(-M_PI/6.0)), 1.0f, 0.0f, ::std::nullopt} }});
        } else { 
            layouts.push_back({"Quad", {{static_cast<float>(dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(dx), static_cast<float>(-dy), 1.0f, 0.0f, ::std::nullopt}}});
            if (::std::abs(dy - static_cast<double>(params.start_offset_m)) < 1e-6 && dx > static_cast<double>(params.start_offset_m)) {
                layouts.push_back({"Collinear Cross", {{static_cast<float>(dx), 0.0f, 1.0f, 0.0f, ::std::nullopt}, {static_cast<float>(-dx), 0.0f, 1.0f, 0.0f, ::std::nullopt}, {0.0f, static_cast<float>(dx), 1.0f, 0.0f, ::std::nullopt}, {0.0f, static_cast<float>(-dx), 1.0f, 0.0f, ::std::nullopt}}});
            }
        }
        return layouts;
    };

    struct TaskInfo { ::std::string name; ::std::vector<Transducer> layout; double dx, dy; int grid_x, grid_y; };
    ::std::vector<TaskInfo> tasks;
    double min_clearance = (2.0 * base_ctx.transducer_radius_m) + base_ctx.transducer_spacing_m;

    for (int i = 0; i < nx; ++i) {
        double dx = static_cast<double>(dx_start + i * params.step_size_m);
        for (int j = 0; j < ny; ++j) {
            double dy = static_cast<double>(dy_start + j * params.step_size_m);
            auto generated = generate_layouts(dx, dy);
            for (const auto& g : generated) {
                bool valid = true;
                for (size_t k = 0; k < g.second.size(); ++k) {
                    for (size_t l = k + 1; l < g.second.size(); ++l) {
                        double dist = ::std::sqrt(::std::pow(g.second[k].x - g.second[l].x, 2) + ::std::pow(g.second[k].y - g.second[l].y, 2));
                        if (dist < min_clearance) { valid = false; break; }
                    }
                    if (!valid) break;
                }
                if (valid) tasks.push_back({g.first, g.second, dx, dy, i, j});
            }
        }
    }

    int total_tasks = tasks.size();
    expected_modes_ = total_tasks * (base_ctx.n_modes * base_ctx.n_modes);
    
    ::std::atomic<int> current_step{0};
    ::std::vector<LayoutResult> candidates;
    ::std::mutex cand_mutex;

    unsigned int max_threads = ::std::max(1u, ::std::thread::hardware_concurrency() - 1);
    ::std::vector<::std::future<void>> futures;

    auto worker = [&](int thread_id) {
        for (int i = thread_id; i < total_tasks; i += max_threads) {
            LayoutResult res = evaluate_layout(tasks[i].layout, base_ctx);
            
            char buf[128];
            ::std::snprintf(buf, sizeof(buf), "%s (dx=%.3f, dy=%.3f)", tasks[i].name.c_str(), tasks[i].dx, tasks[i].dy);
            res.layout_type = ::std::string(buf);
            res.grid_dx = static_cast<float>(tasks[i].dx);
            res.grid_dy = static_cast<float>(tasks[i].dy);
            
            ::std::lock_guard<::std::mutex> lock(cand_mutex);
            
            int grid_idx = tasks[i].grid_y * nx + tasks[i].grid_x;
            heatmap_.scores[grid_idx] = ::std::max(heatmap_.scores[grid_idx], (float)res.alphabet_size);

            if (res.alphabet_size > grid_best_alphabet_) grid_best_alphabet_ = res.alphabet_size;
            candidates.push_back(res);
            
            int step = ++current_step;
            grid_progress_ = (float)step / total_tasks;
        }
    };

    for (unsigned int i = 0; i < max_threads; ++i) futures.push_back(::std::async(::std::launch::async, worker, i));
    for (auto& f : futures) f.wait();

    ::std::sort(candidates.begin(), candidates.end(), [](const LayoutResult& a, const LayoutResult& b) {
        if (a.alphabet_size != b.alphabet_size) return a.alphabet_size > b.alphabet_size;
        return a.total_displacement > b.total_displacement;
    });
    
    if (candidates.size() > 10) candidates.resize(10);
    top_layouts_ = candidates; 
    
    auto_export_grid_results(base_ctx); 

    grid_progress_ = 1.0f;
    is_analyzing_ = false;
    return top_layouts_;
}

void Analyzer::auto_export_grid_results(const SimulationContext& ctx) {
    try {
        ::std::filesystem::create_directories("../sim/transducer_analysis");
        auto now = ::std::chrono::system_clock::now();
        ::std::time_t now_time = ::std::chrono::system_clock::to_time_t(now);
        char timestamp[64];
        ::std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", ::std::localtime(&now_time));

        ::std::string filename = "../sim/transducer_analysis/grid_" + ::std::string(timestamp) + ".json";
        nlohmann::json root;
        
        char display_time[64];
        ::std::strftime(display_time, sizeof(display_time), "%Y-%m-%d %H:%M:%S", ::std::localtime(&now_time));
        root["simulation_date"] = display_time;
        
        nlohmann::json setup;
        setup["transducer_count"] = ctx.transducers.size();
        setup["geometry"] = (ctx.geometry == Geometry::kSquare) ? "Square" : (ctx.geometry == Geometry::kRectangular) ? "Rectangular" : "Circular";
        setup["lx_m"] = ctx.lx; setup["ly_m"] = ctx.ly; setup["thickness_m"] = ctx.h;
        setup["material_e_pa"] = ctx.e; setup["material_rho"] = ctx.rho; setup["n_modes"] = ctx.n_modes;
        setup["transducer_radius_m"] = ctx.transducer_radius_m; setup["transducer_spacing_m"] = ctx.transducer_spacing_m;
        root["setup_parameters"] = setup;

        nlohmann::json tops = nlohmann::json::array();
        for (size_t i = 0; i < top_layouts_.size(); ++i) {
            nlohmann::json layout_json;
            layout_json["rank"] = i + 1;
            layout_json["alphabet_size"] = top_layouts_[i].alphabet_size;
            layout_json["layout_type"] = top_layouts_[i].layout_type;
            layout_json["total_displacement"] = top_layouts_[i].total_displacement;
            layout_json["grid_dx"] = top_layouts_[i].grid_dx;
            layout_json["grid_dy"] = top_layouts_[i].grid_dy;
            
            nlohmann::json coords = nlohmann::json::array();
            for (const auto& t : top_layouts_[i].best_layout) coords.push_back({{"x", t.x}, {"y", t.y}});
            layout_json["transducers"] = coords;
            tops.push_back(layout_json);
        }
        root["top_layouts"] = tops;

        if (heatmap_.valid) {
            nlohmann::json hm_json;
            hm_json["nx"] = heatmap_.nx; hm_json["ny"] = heatmap_.ny;
            hm_json["dx_min"] = heatmap_.dx_min; hm_json["dx_max"] = heatmap_.dx_max;
            hm_json["dy_min"] = heatmap_.dy_min; hm_json["dy_max"] = heatmap_.dy_max;
            hm_json["scores"] = heatmap_.scores;
            root["heatmap_data"] = hm_json;
        }

        ::std::ofstream file(filename);
        if (file.is_open()) file << root.dump(4);
    } catch (...) {}
}

void Analyzer::auto_export_sweep_results(const SimulationContext& ctx) {
    try {
        ::std::filesystem::create_directories("../sim/variable_sweep");
        auto now = ::std::chrono::system_clock::now();
        ::std::time_t now_time = ::std::chrono::system_clock::to_time_t(now);
        char timestamp[64];
        ::std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", ::std::localtime(&now_time));

        ::std::string filename = "../sim/variable_sweep/sweep_" + ::std::string(timestamp) + ".json";
        export_to_json(filename, sweep_results_, ctx);
    } catch (...) {}
}

bool Analyzer::load_sim_results(const ::std::string& filepath, SimulationContext& ctx) {
    (void)ctx; 
    ::std::ifstream file(filepath);
    if (!file.is_open()) return false;
    
    nlohmann::json root;
    try { file >> root; } catch (...) { return false; }

    top_layouts_.clear();
    
    if (root.contains("top_layouts")) {
        for (const auto& item : root["top_layouts"]) {
            LayoutResult res;
            res.alphabet_size = item.value("alphabet_size", 0);
            res.layout_type = item.value("layout_type", "");
            res.total_displacement = item.value("total_displacement", 0.0);
            res.grid_dx = item.value("grid_dx", -1.0f);
            res.grid_dy = item.value("grid_dy", -1.0f);
            
            if (item.contains("transducers")) {
                for (const auto& t_json : item["transducers"]) {
                    Transducer t;
                    t.x = t_json.value("x", 0.0f);
                    t.y = t_json.value("y", 0.0f);
                    t.amplitude = 1.0f;
                    t.phase_rad = 0.0f;
                    res.best_layout.push_back(t);
                }
            }

            if (res.grid_dx < 0.0f && !res.best_layout.empty()) {
                res.grid_dx = static_cast<float>(::std::abs(res.best_layout[0].x));
                res.grid_dy = static_cast<float>(::std::abs(res.best_layout[0].y));
            }
            
            top_layouts_.push_back(res);
        }
    }

    if (root.contains("heatmap_data")) {
        auto hm = root["heatmap_data"];
        heatmap_.nx = hm.value("nx", 0);
        heatmap_.ny = hm.value("ny", 0);
        heatmap_.dx_min = hm.value("dx_min", 0.0f);
        heatmap_.dx_max = hm.value("dx_max", 0.0f);
        heatmap_.dy_min = hm.value("dy_min", 0.0f);
        heatmap_.dy_max = hm.value("dy_max", 0.0f);
        
        if (hm.contains("scores")) {
            heatmap_.scores = hm["scores"].get<::std::vector<float>>();
            heatmap_.valid = true;
        } else { heatmap_.valid = false; }
    } else { heatmap_.valid = false; }
    return true;
}

void Analyzer::load_sweep_results(const ::std::string& filepath, SimulationContext& ctx) {
    (void)ctx;
    ::std::ifstream file(filepath);
    if (!file.is_open()) return;
    
    nlohmann::json root;
    try { file >> root; } catch(...) { return; }

    sweep_results_.clear();
    if (!root.is_array()) return;

    for (const auto& symbol : root) {
        LayoutResult res;
        res.layout_type = symbol.value("display_name", "Loaded");
        res.alphabet_size = 1;
        res.total_displacement = 0.0;
        res.achieved_g = symbol.value("achieved_g", 0.0f);
        res.required_power_w = symbol.value("required_power_w", 0.0f);
        res.required_digital_amp = 1.0f;
        res.is_clipping = false;

        if (symbol.contains("hardware_config") && symbol["hardware_config"].contains("channels")) {
            for (const auto& ch : symbol["hardware_config"]["channels"]) {
                Transducer t;
                t.x = ch.value("x", 0.0);
                t.y = ch.value("y", 0.0);
                t.frequency = ch.value("frequency_hz", 0.0);
                t.amplitude = ch.value("amplitude", 0.0);
                double deg = ch.value("phase_deg", 0.0);
                t.phase_rad = deg * M_PI / 180.0;
                res.best_layout.push_back(t);
            }
        }
        sweep_results_.push_back(res);
    }
}

void Analyzer::export_to_json(const ::std::string& path, const ::std::vector<LayoutResult>& results, const SimulationContext& ctx) { 
    auto round_to = [](double val, int decimals) {
        double p = ::std::pow(10, decimals);
        return ::std::round(val * p) / p;
    };

    nlohmann::json root = nlohmann::json::array();
    int current_id = 1;
    for (const auto& res : results) {
        if (!res.export_selected) continue;
        
        nlohmann::json symbol;
        double freq = res.best_layout.empty() ? 0.0 : res.best_layout[0].frequency.value_or(0.0);
        int freq_int = static_cast<int>(::std::round(freq));
        
        symbol["id"] = current_id++;
        symbol["display_name"] = "CHLADNI_" + ::std::to_string(freq_int);
        
        symbol["achieved_g"] = round_to(res.achieved_g, 2);
        symbol["required_power_w"] = round_to(res.required_power_w, 4); 
        
        nlohmann::json hw_config;
        hw_config["hardware_amp_gain"] = round_to(ctx.hardware_amp_gain, 3);
        hw_config["max_power_w"] = round_to(ctx.transducer_max_power_w, 3);
        
        nlohmann::json channels = nlohmann::json::array();
        for (size_t i = 0; i < res.best_layout.size(); ++i) {
            const auto& t = res.best_layout[i];
            nlohmann::json entry;
            entry["channel"] = static_cast<int>(i + 1);
            entry["x"] = round_to(t.x, 3);
            entry["y"] = round_to(t.y, 3);
            entry["frequency_hz"] = round_to(t.frequency.value_or(0.0), 1);
            
            double phase_deg = t.phase_rad * 180.0 / M_PI;
            int rounded_phase = static_cast<int>(::std::round(phase_deg / 90.0) * 90.0) % 360;
            if (rounded_phase < 0) rounded_phase += 360;
            
            entry["phase_deg"] = rounded_phase;
            entry["amplitude"] = round_to(t.amplitude, 4);
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
    is_analyzing_ = true;
    sweep_progress_ = 0.0f;
    sweep_results_.clear();
    SimulationContext eval_ctx = base_ctx;

    struct ModeInfo {
        double freq;
        Eigen::MatrixXd energy;
        double total_disp;
        ::std::vector<Transducer> layout;
        ::std::string type;
        
        float achieved_g;
        float power_w;
        float req_digital_amp;
        bool clipping;
    };
    ::std::vector<ModeInfo> unique_modes;

    int total_modes = eval_ctx.n_modes * eval_ctx.n_modes;
    int current_mode = 0;

    for (int n = 1; n <= eval_ctx.n_modes; ++n) {
        for (int m = 1; m <= eval_ctx.n_modes; ++m) {
            sweep_progress_ = (float)current_mode++ / total_modes;
            double theoretical_f = physics_->calculate_mode_frequency(n, m, eval_ctx);
            double calibrated_f = (theoretical_f * base_ctx.calib_m) + base_ctx.calib_b;

            if (calibrated_f > base_ctx.max_frequency) continue;
            
            ::std::vector<double> phases = physics_->snipe_phases(n, m, eval_ctx);
            double max_c = 1e-9;
            ::std::vector<double> couplings(eval_ctx.transducers.size());
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                couplings[i] = ::std::abs(PhysicsEngine::transducer_coupling_single(
                    eval_ctx.transducers[i].x, eval_ctx.transducers[i].y, n, m, eval_ctx.lx, eval_ctx.ly, eval_ctx.sign));
                if (couplings[i] > max_c) max_c = couplings[i];
            }

            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].phase_rad = phases[i];
                eval_ctx.transducers[i].amplitude = (couplings[i] / max_c); 
                eval_ctx.transducers[i].frequency = calibrated_f;
            }
            
            double best_f = calibrated_f;
            double max_peak = 0.0;
            
            double window = calibrated_f * 0.05;
            for (int i = 0; i <= 20; ++i) {
                double f = (calibrated_f - window) + (2.0 * window / 20.0 * i);
                Eigen::MatrixXcd resp = physics_->compute_driven_response(f, eval_ctx);
                double peak = resp.array().abs().maxCoeff();
                if (peak > max_peak) { max_peak = peak; best_f = f; }
            }
            
            double fine_center = best_f;
            for (double f = fine_center - 15.0; f <= fine_center + 15.0; f += 0.5) {
                Eigen::MatrixXcd resp = physics_->compute_driven_response(f, eval_ctx);
                double peak = resp.array().abs().maxCoeff();
                if (peak > max_peak) { max_peak = peak; best_f = f; }
            }
            
            double freq = best_f;
            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].frequency = freq;
            }

            Eigen::MatrixXcd resp = physics_->compute_driven_response(freq, eval_ctx);
            Eigen::MatrixXd energy = resp.array().abs();
            double total_disp = energy.sum();
            
            // ====================================================================
            // --- NEW EMPIRICAL AUTO-TUNER ---
            // ====================================================================
            
            // 1. Ask the Stage 5 Polynomial Curve how much power 1G requires at this frequency
            double base_power_1g = (eval_ctx.p_A * freq * freq) + (eval_ctx.p_B * freq) + eval_ctx.p_C;
            if (base_power_1g < 0.01) base_power_1g = 0.01; // Safety floor
            
            // 2. Scale power by the square of the Target G-Force (P scales with G^2)
            double target_g = static_cast<double>(eval_ctx.target_g_force);
            double theoretical_target_power = base_power_1g * (target_g * target_g);
            
            // 3. Compensate for layout inefficiency. 
            // If max_coupling is 0.5 compared to the ideal center, it takes 4x the power.
            double spatial_penalty = 1.0 / (max_c * max_c);
            double actual_required_power = theoretical_target_power * spatial_penalty;
            
            // 4. Calculate required Digital Amp based on Hardware Gain Knob
            // Power = Max_W * (Digital_Amp * HW_Gain)^2
            // Digital_Amp = sqrt(Power / Max_W) / HW_Gain
            double req_digital_amp = ::std::sqrt(actual_required_power / eval_ctx.transducer_max_power_w) / eval_ctx.hardware_amp_gain;
            
            bool clipped = (req_digital_amp > 1.0);
            double final_digital_amp = clipped ? 1.0 : req_digital_amp;
            
            // If clipping, reverse-calculate what G-Force we ACTUALLY achieved at 1.0 amp
            double final_achieved_g = target_g;
            if (clipped) {
                double max_possible_power = eval_ctx.transducer_max_power_w * (eval_ctx.hardware_amp_gain * eval_ctx.hardware_amp_gain);
                double power_ratio = max_possible_power / actual_required_power;
                final_achieved_g = target_g * ::std::sqrt(power_ratio);
            }

            for (size_t i = 0; i < eval_ctx.transducers.size(); ++i) {
                eval_ctx.transducers[i].amplitude = (couplings[i] / max_c) * final_digital_amp;
            }
            
            // ====================================================================

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
                unique_modes.push_back({freq, energy, total_disp, eval_ctx.transducers, "Mode_" + ::std::to_string(n) + "_" + ::std::to_string(m), 
                                        static_cast<float>(final_achieved_g), static_cast<float>(actual_required_power), static_cast<float>(req_digital_amp), clipped});
            }
        }
    }
    
    for (const auto& m : unique_modes) {
        LayoutResult res;
        res.best_layout = m.layout;
        res.alphabet_size = 1;
        res.layout_type = m.type;
        res.total_displacement = m.total_disp;
        res.achieved_g = m.achieved_g;
        res.required_power_w = m.power_w;
        res.required_digital_amp = m.req_digital_amp;
        res.is_clipping = m.clipping;
        sweep_results_.push_back(res);
    }

    ::std::sort(sweep_results_.begin(), sweep_results_.end(), [](const LayoutResult& a, const LayoutResult& b) {
        return a.best_layout[0].frequency.value_or(0.0) < b.best_layout[0].frequency.value_or(0.0);
    });

    auto_export_sweep_results(base_ctx);

    sweep_progress_ = 1.0f;
    is_analyzing_ = false;
    return sweep_results_; 
}

::std::vector<LayoutResult> Analyzer::run_grid_search(const SimulationContext& base_ctx) { 
    (void)base_ctx; 
    return {}; 
}

} // namespace chladni