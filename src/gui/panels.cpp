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
#define M_PI 3.14159265358979323846
#endif

namespace chladni {

void Panels::draw_main_ui(SimulationContext& ctx, Application* app, Analyzer& analyzer, float& current_freq, float& f_max, bool& is_batch_running, float& batch_progress, ::std::string& batch_status) {
    ImGui::Begin("Workflow Pipeline");
    if (ImGui::BeginTabBar("Stages")) {
        if (ImGui::BeginTabItem("Stage 1: Manual")) {
            draw_stage1_manual(ctx, app, current_freq, f_max);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 2: Symmetric Explorer")) {
            draw_stage2_grid(ctx, analyzer);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 3: Variable Sweep")) {
            draw_stage3_sweep(ctx, analyzer);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Stage 4: Batch Plotter")) {
            draw_stage4_batch(ctx, app, is_batch_running, batch_progress, batch_status);
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
        ImGui::InputFloat("Max Range", &f_max);
        if (ImGui::Button("Prev Peak")) app->snap_to_resonance(-1);
        ImGui::SameLine();
        if (ImGui::Button("Next Peak")) app->snap_to_resonance(1);
    }

    if (ImGui::CollapsingHeader("SiniLink Amplifiers", ImGuiTreeNodeFlags_DefaultOpen)) {
        double min_v = 0.0, max_v = 1.0;
        ImGui::SliderScalar("Amp 1 Gain", ImGuiDataType_Double, &ctx.base_volume_1, &min_v, &max_v);
        ImGui::SliderScalar("Amp 2 Gain", ImGuiDataType_Double, &ctx.base_volume_2, &min_v, &max_v);
    }

    if (ImGui::CollapsingHeader("Transducers", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("1-Center")) app->apply_preset("1-center");
        ImGui::SameLine();
        if (ImGui::Button("4-Corners")) app->apply_preset("4-corners");
        
        int t_count = static_cast<int>(ctx.transducers.size());
        if (ImGui::SliderInt("Count", &t_count, 1, 4)) {
            while (ctx.transducers.size() < (size_t)t_count) ctx.transducers.push_back({0.0, 0.0, 1.0, 0.0, ::std::nullopt});
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
                ctx.transducers[i].amplitude = ::std::clamp(ctx.transducers[i].amplitude, 0.0, 1.0);
                
                float deg = (float)(ctx.transducers[i].phase_rad * 180.0 / M_PI);
                if (ImGui::SliderFloat("Phase (°)", &deg, 0, 360)) ctx.transducers[i].phase_rad = (double)deg * M_PI / 180.0;
                double deg_d = (double)deg;
                if (ImGui::InputDouble("Phase (°) ##Text", &deg_d)) ctx.transducers[i].phase_rad = deg_d * M_PI / 180.0;
                
                if (changed) app->get_physics()->clamp_transducer(ctx.transducers[i], ctx);
                
                ImGui::TreePop();
            }
        }
    }
}

void Panels::draw_stage2_grid(SimulationContext& ctx, Analyzer& analyzer) {
    static GridParams params;
    static ::std::vector<LayoutResult> top_layouts;
    static ::std::future<::std::vector<LayoutResult>> grid_future;
    static bool is_running = false;

    double lx = ctx.lx;
    double ly = (ctx.geometry == Geometry::kCircular) ? ctx.lx : ctx.ly;
    double dx_start = 0.025, dx_end = lx / 2.0 - params.edge_gap_m;
    double dy_start = 0.025, dy_end = ly / 2.0 - params.edge_gap_m;
    
    int nx = (dx_end < dx_start) ? 0 : static_cast<int>((dx_end - dx_start)/params.step_size_m + 1);
    int ny = (dy_end < dy_start) ? 0 : static_cast<int>((dy_end - dy_start)/params.step_size_m + 1);
    int total_iterations = nx * ny;

    if (ImGui::CollapsingHeader("Explorer Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        double step_mm = params.step_size_m * 1000.0;
        if (ImGui::InputDouble("Step Size (mm)", &step_mm)) params.step_size_m = ::std::max(1.0, step_mm) / 1000.0;
        
        double gap_mm = params.edge_gap_m * 1000.0;
        if (ImGui::InputDouble("Edge Gap (mm)", &gap_mm)) params.edge_gap_m = ::std::max(0.0, gap_mm) / 1000.0;
        
        ImGui::Text("Estimated Iterations: %d (%d x %d grid)", total_iterations, nx, ny);
    }

    if (is_running) {
        if (grid_future.wait_for(::std::chrono::seconds(0)) == ::std::future_status::ready) {
            top_layouts = grid_future.get();
            is_running = false;
        }
        char overlay[128];
        ::std::snprintf(overlay, sizeof(overlay), "Best Alphabet: %d", analyzer.get_grid_best_alphabet());
        ImGui::ProgressBar(analyzer.get_grid_progress(), ImVec2(-1, 0), overlay);
        if (ImGui::Button("Cancel Search", ImVec2(-1, 0))) {
            is_running = false; 
        }
    } else {
        if (ImGui::Button("Run Symmetric Grid Search", ImVec2(-1, 40))) {
            GridParams p = params;
            grid_future = ::std::async(::std::launch::async, [&analyzer, ctx, p]() {
                return analyzer.run_symmetric_grid_search(ctx, p);
            });
            is_running = true;
        }
    }

    if (!top_layouts.empty()) {
        ImGui::Separator();
        ImGui::Text("Top Discovered Symmetric Layouts:");
        for (size_t i = 0; i < top_layouts.size(); ++i) {
            ImGui::Text("Layout #%d (Alphabet: %d)", (int)i+1, top_layouts[i].alphabet_size);
            ImGui::SameLine();
            if (ImGui::Button(("Preview on Plate##" + ::std::to_string(i)).c_str())) {
                ctx.transducers = top_layouts[i].best_layout;
            }
            ImGui::SameLine();
            if (ImGui::Button(("Lock In Layout##" + ::std::to_string(i)).c_str())) {
                ctx.transducers = top_layouts[i].best_layout;
            }
        }
    }
}

void Panels::draw_stage3_sweep(SimulationContext& ctx, Analyzer& analyzer) {
    ImGui::Text("Frequency Sensitivity Sweep");
    ImGui::Text("Layout locked with %d transducers.", (int)ctx.transducers.size());
    
    static ::std::vector<LayoutResult> sweep_results;
    static ::std::future<::std::vector<LayoutResult>> sweep_future;
    static bool is_sweeping = false;

    if (is_sweeping) {
        if (sweep_future.wait_for(::std::chrono::seconds(0)) == ::std::future_status::ready) {
            sweep_results = sweep_future.get();
            is_sweeping = false;
        }
        ImGui::ProgressBar(analyzer.get_sweep_progress(), ImVec2(-1, 0), "Sweeping Modes...");
    } else {
        if (ImGui::Button("Run Sensitivity Sweep", ImVec2(-1, 40))) {
            sweep_future = ::std::async(::std::launch::async, [&analyzer, ctx]() {
                return analyzer.run_sensitivity_sweep(ctx);
            });
            is_sweeping = true;
        }
    }

    if (!sweep_results.empty()) {
        if (ImGui::Button("Export master_symbols.json", ImVec2(-1, 40))) {
            analyzer.export_to_json("../master_symbols.json", sweep_results, ctx);
        }

        if (ImGui::BeginTable("SweepResults", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Mode");
            ImGui::TableSetupColumn("Freq (Hz)");
            ImGui::TableSetupColumn("Amp (W)");
            ImGui::TableSetupColumn("Action");
            ImGui::TableHeadersRow();

            for (auto& res : sweep_results) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%s", res.layout_type.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.1f", res.best_layout[0].frequency.value_or(0.0));
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", res.best_layout[0].amplitude);
                ImGui::TableSetColumnIndex(3);
                if (ImGui::Button(("Apply##" + res.layout_type).c_str())) {
                    ctx.transducers = res.best_layout;
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
