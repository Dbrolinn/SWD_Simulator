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
        if (ImGui::BeginTabItem("Stage 2: GA Optimizer")) {
            draw_stage2_ga(ctx, analyzer);
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
        ImGui::InputDouble("Lx (m)", &ctx.lx);
        if (ctx.geometry == Geometry::kRectangular) ImGui::InputDouble("Ly (m)", &ctx.ly);
        ImGui::InputDouble("Thickness (m)", &ctx.h);
    }

    if (ImGui::CollapsingHeader("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Frequency (Hz)", &current_freq, 20.0f, f_max);
        ImGui::InputFloat("Max Range", &f_max);
        if (ImGui::Button("Prev Peak")) app->snap_to_resonance(-1);
        ImGui::SameLine();
        if (ImGui::Button("Next Peak")) app->snap_to_resonance(1);
    }

    if (ImGui::CollapsingHeader("Transducers", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("1-Center")) app->apply_preset("1-center");
        ImGui::SameLine();
        if (ImGui::Button("4-Corners")) app->apply_preset("4-corners");
        
        for (size_t i = 0; i < ctx.transducers.size(); ++i) {
            ::std::string label = "T" + ::std::to_string(i+1);
            if (ImGui::TreeNode(label.c_str())) {
                bool changed = false;
                changed |= ImGui::InputDouble("X", &ctx.transducers[i].x);
                changed |= ImGui::InputDouble("Y", &ctx.transducers[i].y);
                
                double min_p = 0.0, max_p = 25.0;
                ImGui::SliderScalar("Power (W)", ImGuiDataType_Double, &ctx.transducers[i].amplitude, &min_p, &max_p);
                
                float deg = (float)(ctx.transducers[i].phase_rad * 180.0 / M_PI);
                if (ImGui::SliderFloat("Phase (°)", &deg, 0, 360)) ctx.transducers[i].phase_rad = (double)deg * M_PI / 180.0;
                
                if (changed) app->get_physics()->clamp_transducer(ctx.transducers[i], ctx);
                
                ImGui::TreePop();
            }
        }
    }
}

void Panels::draw_stage2_ga(SimulationContext& ctx, Analyzer& analyzer) {
    static GAParams params;
    ImGui::InputInt("Population", &params.population_size);
    ImGui::InputInt("Generations", &params.generations);
    ImGui::SliderInt("Transducers", &params.transducer_count, 4, 6);
    
    static char path[256] = "ga_optimal_layouts.csv";
    ImGui::InputText("Export Path", path, 256);
    params.export_path = path;

    if (ImGui::Button("Run Genetic Algorithm Optimization")) {
        analyzer.run_genetic_algorithm(ctx, params);
    }
}

void Panels::draw_stage3_sweep(SimulationContext& ctx, Analyzer& analyzer) {
    ImGui::Text("Exhaustive Sensitivity Analysis");
    ImGui::BulletText("Sweep Phases: 0, 90, 180, 270");
    ImGui::BulletText("Sweep Power: 10W, 15W, 20W, 25W");
    
    if (ImGui::Button("Run Exhaustive Parameter Sweep")) {
        analyzer.run_sensitivity_sweep(ctx);
    }
}

void Panels::draw_stage4_batch(SimulationContext& ctx, Application* app, bool& is_batch_running, float& batch_progress, ::std::string& batch_status) {
    ImGui::Text("Automated Plotting Suite");
    
    static char csv_path[256] = "true_alphabet_modes.csv";
    ImGui::InputText("Input CSV", csv_path, 256);
    
    static char out_dir[256] = "./output_images";
    ImGui::InputText("Output Dir", out_dir, 256);

    if (is_batch_running) {
        ImGui::Text("Status: %s", batch_status.c_str());
        ImGui::ProgressBar(batch_progress, ImVec2(-1, 0));
    } else {
        if (ImGui::Button("Start Batch Generation")) {
            app->start_batch_plotting(csv_path, out_dir);
        }
    }
}

void Panels::draw_material_selector(SimulationContext& ctx) {
    if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen)) {
        static const char* materials[] = { "Aluminium", "Steel", "Brass", "Glass" };
        static int current_mat = 0;
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
