/**
 * @file panels.cpp
 * @brief UI panels for the Chladni simulator.
 */

#include "panels.h"
#include "application.h"
#include "imgui.h"
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace chladni {

void Panels::draw_main_ui(SimulationContext& ctx, Application* app, Analyzer& analyzer, float& current_freq, float& f_max, bool& is_batch_running, float& batch_progress, ::std::string& batch_status) {
    ImGui::SetNextWindowSize(ImVec2(450, 600), ImGuiCond_FirstUseEver);
    ImGui::Begin("Workflow Pipeline");
    if (ImGui::BeginTabBar("Stages", ImGuiTabBarFlags_FittingPolicyScroll)) {
        if (ImGui::BeginTabItem("Stage 1: Manual")) {
            draw_stage1_manual(ctx, app, current_freq, f_max);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 2: Symmetric Explorer")) {
            draw_stage2_grid(ctx, analyzer, app);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 3: Variable Sweep")) {
            draw_stage3_sweep(ctx, analyzer, current_freq);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 4: Batch Plotter")) {
            draw_stage4_batch(ctx, app, is_batch_running, batch_progress, batch_status);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 5: Lab Calibration")) {
            draw_stage5_calibration(ctx);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void Panels::draw_stage1_manual(SimulationContext& ctx, Application* app, float& current_freq, float& f_max) {
    draw_material_selector(ctx);
    
    if (ImGui::CollapsingHeader("Plate Control", ImGuiTreeNodeFlags_DefaultOpen)) {
        static const char* items[] = {"Square", "Rectangular", "Circular"};
        int current = (int)ctx.geometry;
        if (ImGui::Combo("Geometry", &current, items, 3)) {
            ctx.geometry = (Geometry)current;
        }
        
        double lx_mm = ctx.lx * 1000.0;
        if (ImGui::InputDouble("Lx (mm)", &lx_mm)) ctx.lx = lx_mm / 1000.0;
        
        if (ctx.geometry == Geometry::kRectangular) {
            double ly_mm = ctx.ly * 1000.0;
            if (ImGui::InputDouble("Ly (mm)", &ly_mm)) ctx.ly = ly_mm / 1000.0;
        }
        
        double h_mm = ctx.h * 1000.0;
        if (ImGui::InputDouble("Thickness (mm)", &h_mm)) ctx.h = h_mm / 1000.0;
    }

    if (ImGui::CollapsingHeader("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Frequency (Hz)", &current_freq, 20.0f, f_max);
        ImGui::InputFloat("Sweep Max Limit", &f_max);
        
        ImGui::Separator();
        
        int n_modes = ctx.n_modes;
        if (ImGui::SliderInt("Modes Resolution", &n_modes, 5, 40)) {
            ctx.n_modes = n_modes;
        }
        ImGui::InputDouble("Hardware Cutoff Limit (Hz)", &ctx.max_frequency);

        ImGui::Separator();
        if (ImGui::Button("Prev Peak")) app->snap_to_resonance(-1);
        ImGui::SameLine();
        if (ImGui::Button("Next Peak")) app->snap_to_resonance(1);
    }

    // FIXED: Completely removed the redundant "SiniLink Amplifiers" section from Stage 1!
    // Hardware constraints are now exclusively controlled mathematically in Stage 3.

    if (ImGui::CollapsingHeader("Transducers", ImGuiTreeNodeFlags_DefaultOpen)) {
        
        double tr_mm = ctx.transducer_radius_m * 1000.0;
        if (ImGui::InputDouble("Transducer Radius (mm)", &tr_mm)) ctx.transducer_radius_m = ::std::max(1.0, tr_mm) / 1000.0;
        
        double ts_mm = ctx.transducer_spacing_m * 1000.0;
        if (ImGui::InputDouble("Min Spacing Gap (mm)", &ts_mm)) ctx.transducer_spacing_m = ::std::max(0.0, ts_mm) / 1000.0;

        ImGui::Separator();

        if (ImGui::Button("1-Center")) app->apply_preset("1-center");
        ImGui::SameLine();
        if (ImGui::Button("4-Corners")) app->apply_preset("4-corners");
        
        int t_count = static_cast<int>(ctx.transducers.size());
        if (ImGui::SliderInt("Count", &t_count, 1, 4)) {
            while (ctx.transducers.size() < (size_t)t_count) ctx.transducers.push_back({0.0f, 0.0f, 1.0f, 0.0f, ::std::nullopt});
            while (ctx.transducers.size() > (size_t)t_count) ctx.transducers.pop_back();
        }

        for (size_t i = 0; i < ctx.transducers.size(); ++i) {
            ::std::string label = "T" + ::std::to_string(i+1);
            if (ImGui::TreeNode(label.c_str())) {
                bool changed = false;
                double tx_mm = ctx.transducers[i].x * 1000.0;
                double ty_mm = ctx.transducers[i].y * 1000.0;
                
                if (ImGui::InputDouble("X (mm)", &tx_mm)) { ctx.transducers[i].x = tx_mm / 1000.0; changed = true; }
                if (ImGui::InputDouble("Y (mm)", &ty_mm)) { ctx.transducers[i].y = ty_mm / 1000.0; changed = true; }
                
                double min_p = 0.0, max_p = 1.0;
                ImGui::SliderScalar("Digital Amp", ImGuiDataType_Double, &ctx.transducers[i].amplitude, &min_p, &max_p);
                ImGui::InputDouble("Digital Amp ##Text", &ctx.transducers[i].amplitude);
                
                // FIXED: Use pure double limits (0.0, 1.0) instead of float limits (0.0f, 1.0f)
                ctx.transducers[i].amplitude = ::std::clamp(ctx.transducers[i].amplitude, 0.0, 1.0);
                
                double deg = (ctx.transducers[i].phase_rad * 180.0 / M_PI);
                double min_deg = 0.0, max_deg = 360.0;
                if (ImGui::SliderScalar("Phase (°)", ImGuiDataType_Double, &deg, &min_deg, &max_deg)) ctx.transducers[i].phase_rad = deg * M_PI / 180.0;
                if (ImGui::InputDouble("Phase (°) ##Text", &deg)) ctx.transducers[i].phase_rad = deg * M_PI / 180.0;
                
                if (changed) app->get_physics()->clamp_transducer(ctx.transducers[i], ctx);
                
                ImGui::TreePop();
            }
        }
    }
}

void Panels::draw_stage2_grid(SimulationContext& ctx, Analyzer& analyzer, Application* app) {
    GridParams& params = app->stage2_params_; 
    
    float active_radius_m = static_cast<float>(ctx.transducer_radius_m);
    float min_spacing_m = static_cast<float>(ctx.transducer_spacing_m);
    float absolute_min_offset = active_radius_m + (min_spacing_m / 2.0f);
    
    if (params.start_offset_m < absolute_min_offset) params.start_offset_m = absolute_min_offset;
    if (params.edge_gap_m < 0.0f) params.edge_gap_m = 0.0f;
    
    static ::std::future<::std::vector<LayoutResult>> grid_future;

    if (ImGui::CollapsingHeader("Explorer Mode Control", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (params.use_roi) {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "ACTIVE: FOCUSED ROI SWEEP");
            ImGui::Text("X Bounds: [%.3f, %.3f] m", params.roi_dx_min, params.roi_dx_max);
            ImGui::Text("Y Bounds: [%.3f, %.3f] m", params.roi_dy_min, params.roi_dy_max);
            ImGui::Spacing();
            if (ImGui::Button("Clear ROI / Return to Full Plate", ImVec2(-1, 0))) {
                params.use_roi = false;
                params.step_size_m = 0.015f; 
            }
        } else {
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "ACTIVE: FULL PLATE MACRO SWEEP");
            
            float offset_mm = params.start_offset_m * 1000.0f;
            if (ImGui::InputFloat("Keep-out Core Radius (mm)", &offset_mm)) params.start_offset_m = ::std::max(absolute_min_offset, offset_mm / 1000.0f);
            
            float gap_mm = params.edge_gap_m * 1000.0f;
            if (ImGui::InputFloat("Plate Edge Gap Margin (mm)", &gap_mm)) params.edge_gap_m = ::std::max(0.0f, gap_mm / 1000.0f);
            
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Note: Use Heatmap tab to generate ROI.");
        }
        
        ImGui::Separator();
        float step_mm = params.step_size_m * 1000.0f;
        if (ImGui::InputFloat("Resolution Step Size (mm)", &step_mm)) params.step_size_m = ::std::max(1.0f, step_mm) / 1000.0f;
    }

    float lx = static_cast<float>(ctx.lx);
    float ly = static_cast<float>((ctx.geometry == Geometry::kCircular) ? ctx.lx : ctx.ly);
    
    float dx_start = params.use_roi ? params.roi_dx_min : params.start_offset_m;
    float dy_start = params.use_roi ? params.roi_dy_min : params.start_offset_m;
    float dx_end = params.use_roi ? params.roi_dx_max : (lx / 2.0f - params.edge_gap_m - active_radius_m);
    float dy_end = params.use_roi ? params.roi_dy_max : (ly / 2.0f - params.edge_gap_m - active_radius_m);
    
    if (dx_end < dx_start) dx_end = dx_start;
    if (dy_end < dy_start) dy_end = dy_start;

    int nx = ::std::max(1, (int)::std::floor((dx_end - dx_start) / params.step_size_m) + 1);
    int ny = ::std::max(1, (int)::std::floor((dy_end - dy_start) / params.step_size_m) + 1);
    int layouts_per_point = (ctx.transducers.size() == 2) ? 4 : (ctx.transducers.size() == 4 ? 2 : 1);
    if (ctx.transducers.size() == 3) layouts_per_point = 1;
    
    int total_layouts_est = nx * ny * layouts_per_point;
    int expected_modes_est = total_layouts_est * (ctx.n_modes * ctx.n_modes);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Simulation Predictor Dashboard:");
    ImGui::Text("Search Grid: %d x %d nodes", nx, ny);
    ImGui::Text("Layouts to Evaluate: ~%d variations", total_layouts_est);
    ImGui::Text("Complexity: ~%d distinct Matrix Solves", expected_modes_est);
    ImGui::Separator();

    if (analyzer.is_analyzing()) {
        char overlay1[128]; ::std::snprintf(overlay1, sizeof(overlay1), "Matrix Positions Analyzed: %.1f%%", analyzer.get_grid_progress() * 100.0f);
        ImGui::ProgressBar(analyzer.get_grid_progress(), ImVec2(-1, 0), overlay1);

        char overlay2[128]; ::std::snprintf(overlay2, sizeof(overlay2), "Mode Calculations: %.1f%%", analyzer.get_grid_sub_progress() * 100.0f);
        ImGui::ProgressBar(analyzer.get_grid_sub_progress(), ImVec2(-1, 0), overlay2);

        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Best Alphabet Discovered: %d symbols", analyzer.get_grid_best_alphabet());
    } else {
        if (grid_future.valid() && grid_future.wait_for(::std::chrono::seconds(0)) == ::std::future_status::ready) {
            grid_future.get(); 
        }

        if (ImGui::Button(params.use_roi ? "Execute Fine Sweep on ROI" : "Execute Coarse Map of Full Plate", ImVec2(-1, 40))) {
            GridParams p = params;
            grid_future = ::std::async(::std::launch::async, [&analyzer, ctx, p]() {
                return analyzer.run_symmetric_grid_search(ctx, p);
            });
        }
    }

    const auto& top_layouts = analyzer.get_top_layouts();
    if (!top_layouts.empty()) {
        ImGui::Separator();
        ImGui::Text("Top Discovered Layout Configurations:");
        for (size_t i = 0; i < top_layouts.size(); ++i) {
            ImGui::Text("Rank #%d: Alphabet: %d | Config: %s", (int)i+1, top_layouts[i].alphabet_size, top_layouts[i].layout_type.c_str());
            ImGui::SameLine();
            if (ImGui::Button(("Push to Hardware##" + ::std::to_string(i)).c_str())) {
                ctx.transducers = top_layouts[i].best_layout;
            }
        }
    }
}

void Panels::draw_stage3_sweep(SimulationContext& ctx, Analyzer& analyzer, float& current_freq) {
    ImGui::Text("Frequency Sensitivity Sweep & Auto-Tuner");
    ImGui::Text("Layout locked with %d transducers.", (int)ctx.transducers.size());
    
    // FIXED: Upgraded InputFloat to InputDouble to match the physics engine's precision
    if (ImGui::CollapsingHeader("Auto-Tuner Target Goals", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputDouble("Amplifier Max Power (W)", &ctx.transducer_max_power_w);
        ImGui::InputDouble("Hardware Gain Knob (0.0 - 1.0)", &ctx.hardware_amp_gain);
        ImGui::InputDouble("Target Plate Acceleration (G)", &ctx.target_g_force);
        ImGui::InputDouble("Reference Sand Mass (mg)", &ctx.particle_mass_mg);
        ImGui::Separator();
    }
    
    static ::std::vector<LayoutResult> sweep_results;
    static ::std::future<::std::vector<LayoutResult>> sweep_future;

    if (analyzer.is_analyzing()) {
        ImGui::ProgressBar(analyzer.get_sweep_progress(), ImVec2(-1, 0), "Sweeping Modes...");
    } else {
        if (sweep_future.valid() && sweep_future.wait_for(::std::chrono::seconds(0)) == ::std::future_status::ready) {
            sweep_results = sweep_future.get();
        }

        if (ImGui::Button("Run Sensitivity Sweep", ImVec2(-1, 40))) {
            sweep_future = ::std::async(::std::launch::async, [&analyzer, ctx]() {
                return analyzer.run_sensitivity_sweep(ctx);
            });
        }
    }

    if (!sweep_results.empty()) {
        if (ImGui::Button("Export master_symbols.json", ImVec2(-1, 40))) {
            analyzer.export_to_json("../master_symbols.json", sweep_results, ctx);
        }

        // FIXED: Expanded 5-column AGC Display showing actual theoretical power
        if (ImGui::BeginTable("SweepResults", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 300))) {
            ImGui::TableSetupColumn("Freq (Hz)");
            ImGui::TableSetupColumn("Digital Amp");
            ImGui::TableSetupColumn("G Force");
            ImGui::TableSetupColumn("Req. Power");
            ImGui::TableSetupColumn("Action");
            ImGui::TableHeadersRow();

            for (auto& res : sweep_results) {
                ImGui::TableNextRow();
                
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%.1f", res.best_layout[0].frequency.value_or(0.0));
                
                ImGui::TableSetColumnIndex(1);
                if (res.is_clipping) {
                    // Symbol cannot reach target force. Tell user what software volume would technically be required.
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Req: %.2f", res.required_digital_amp);
                } else {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "%.2f", res.required_digital_amp);
                }

                ImGui::TableSetColumnIndex(2);
                if (res.is_clipping) {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "%.1f (Max)", res.achieved_g);
                } else {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "%.1f (OK)", res.achieved_g);
                }

                ImGui::TableSetColumnIndex(3);
                if (res.is_clipping) {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "%.1f W", res.required_power_w);
                } else {
                    ImGui::Text("%.1f W", res.required_power_w);
                }

                ImGui::TableSetColumnIndex(4);
                if (ImGui::Button(("Apply##" + res.layout_type).c_str())) {
                    ctx.transducers = res.best_layout;
                    current_freq = static_cast<float>(res.best_layout[0].frequency.value_or(0.0));
                }
            }
            ImGui::EndTable();
        }
    }
}

void Panels::draw_stage4_batch(SimulationContext& ctx, Application* app, bool& is_batch_running, float& batch_progress, ::std::string& batch_status) {
    (void)ctx;
    ImGui::Text("Automated Plotting Suite");
    
    if (is_batch_running) {
        ImGui::Text("Status: %s", batch_status.c_str());
        ImGui::ProgressBar(batch_progress, ImVec2(-1, 0));
    } else {
        if (ImGui::Button("Render from master_symbols.json", ImVec2(-1, 40))) {
            app->start_batch_plotting("../master_symbols.json", "../dictionary");
        }
    }
}

void Panels::draw_stage5_calibration(SimulationContext& ctx) {
    ImGui::Text("Lab Calibration Module");
    ImGui::Separator();
    
    struct CalibPoint {
        double theoretical;
        double actual;
    };
    static ::std::vector<CalibPoint> points;
    
    if (ImGui::Button("Add Data Point")) {
        points.push_back({0.0, 0.0});
    }
    
    if (ImGui::BeginTable("CalibTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Theoretical (Hz)");
        ImGui::TableSetupColumn("Actual Lab (Hz)");
        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        
        for (size_t i = 0; i < points.size(); ++i) {
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(i));
            
            ImGui::TableSetColumnIndex(0);
            ImGui::SetNextItemWidth(-1);
            ImGui::InputDouble("##Theo", &points[i].theoretical);
            
            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-1);
            ImGui::InputDouble("##Actual", &points[i].actual);
            
            ImGui::TableSetColumnIndex(2);
            if (ImGui::Button("Remove")) {
                points.erase(points.begin() + i);
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    
    if (ImGui::Button("Calculate Calibration", ImVec2(-1, 40))) {
        if (points.size() >= 2) {
            double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
            for (const auto& p : points) {
                sum_x += p.theoretical;
                sum_y += p.actual;
                sum_xy += p.theoretical * p.actual;
                sum_x2 += p.theoretical * p.theoretical;
            }
            double n = static_cast<double>(points.size());
            double denominator = (n * sum_x2 - sum_x * sum_x);
            if (::std::abs(denominator) > 1e-9) {
                // FIXED: Removed the static_cast<float> since calib_m and calib_b are now doubles
                ctx.calib_m = (n * sum_xy - sum_x * sum_y) / denominator;
                ctx.calib_b = (sum_y - ctx.calib_m * sum_x) / n;
            }
        }
    }
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Active Calibration:");
    ImGui::Text("f_actual = %.6f * f_theoretical + %.2f", ctx.calib_m, ctx.calib_b);
    
    if (ImGui::Button("Reset to Ideal (1.0, 0.0)")) {
        ctx.calib_m = 1.0;
        ctx.calib_b = 0.0;
    }
}

void Panels::draw_material_selector(SimulationContext& ctx) {
    if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen)) {
        static const char* materials[] = { "Aluminium", "Steel", "Brass", "Glass" };
        static int current_mat = 1;
        if (ImGui::Combo("Preset", &current_mat, materials, 4)) {
            if (current_mat == 0) { ctx.e = 69e9; ctx.rho = 2700.0; ctx.nu = 0.33; }
            else if (current_mat == 1) { ctx.e = 193e9; ctx.rho = 8000.0; ctx.nu = 0.29; }
        }
        ImGui::InputDouble("E (Pa)", &ctx.e);
        ImGui::InputDouble("rho (kg/m3)", &ctx.rho);
        ImGui::InputDouble("nu", &ctx.nu);
    }
}

} // namespace chladni