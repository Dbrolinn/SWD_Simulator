/**
 * @file application.cpp
 * @brief Main application window and ImGui context management.
 */

#include "application.h"
#include "panels.h"
#include "gui/gl_custom_loader.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <filesystem>
#include <cstring>
#include <fstream>
#include <cfloat>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// OpenGL
#include <GLFW/glfw3.h>

// ImGui
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

namespace chladni {

Application::Application(const ::std::string& title, int width, int height)
    : window_(nullptr), title_(title), width_(width), height_(height) {
  
  physics_ = ::std::make_shared<PhysicsEngine>(200);
  analyzer_ = ::std::make_shared<Analyzer>(physics_);

  ctx_.geometry = Geometry::kRectangular;
  ctx_.lx = 0.30;
  ctx_.ly = 0.20;
  ctx_.h = 0.0010;
  ctx_.e = 193e9; 
  ctx_.rho = 8000.0;
  ctx_.nu = 0.29;
  ctx_.damping = 0.005;
  ctx_.n_modes = 15;
  ctx_.max_frequency = 20000.0;
  ctx_.sign = 1;
  
  ctx_.calib_m = 1.0;
  ctx_.calib_b = 0.0;
  
  // Power calibration defaults
  ctx_.p_A = 0.0;
  ctx_.p_B = 0.0;
  ctx_.p_C = 1.0;
  
  ctx_.transducer_max_power_w = 25.0f;
  ctx_.hardware_amp_gain = 0.8f;
  ctx_.target_g_force = 5.0f;
  ctx_.particle_mass_mg = 1.0f;
  
  ctx_.transducer_radius_m = 0.025;
  ctx_.transducer_spacing_m = 0.005;
  ctx_.speaker = {};
  
  ctx_.transducers.clear();
  for (int i = 0; i < 4; ++i) ctx_.transducers.push_back({0.0, 0.0, 1.0, 0.0, ::std::nullopt});
}

Application::~Application() { shutdown(); }

bool Application::init() {
  if (!glfwInit()) return false;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
  if (!window_) { glfwTerminate(); return false; }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  glGenTextures(1, &plate_texture_);
  glBindTexture(GL_TEXTURE_2D, plate_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  refresh_sim_files();

  return true;
}

void Application::refresh_sim_files() {
    available_sim_files_.clear();
    try {
        ::std::filesystem::create_directories("../sim/transducer_analysis");
        for (const auto& entry : ::std::filesystem::directory_iterator("../sim/transducer_analysis")) {
            if (entry.path().extension() == ".json") {
                available_sim_files_.push_back(entry.path().filename().string());
            }
        }
        ::std::sort(available_sim_files_.rbegin(), available_sim_files_.rend());
    } catch (...) {}
}

void Application::shutdown() {
  if (plate_texture_) glDeleteTextures(1, &plate_texture_);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  if (window_) glfwDestroyWindow(window_);
  glfwTerminate();
}

void Application::update_texture() {
  static double last_rendered_f = -1.0;
  static ::std::vector<Transducer> last_layout;
  
  bool changed = ::std::abs(last_rendered_f - current_freq_) > 1e-4;
  if (!changed) {
      if (last_layout.size() != ctx_.transducers.size()) {
          changed = true;
      } else {
          for (size_t i = 0; i < ctx_.transducers.size(); ++i) {
              if (::std::abs(ctx_.transducers[i].x - last_layout[i].x) > 1e-6 ||
                  ::std::abs(ctx_.transducers[i].y - last_layout[i].y) > 1e-6 ||
                  ::std::abs(ctx_.transducers[i].amplitude - last_layout[i].amplitude) > 1e-6 ||
                  ::std::abs(ctx_.transducers[i].phase_rad - last_layout[i].phase_rad) > 1e-6) {
                  changed = true;
                  break;
              }
          }
      }
  }

  if (!changed && !current_sand_.isZero()) return;

  physics_->compute_visuals(static_cast<double>(current_freq_), ctx_, current_sand_, current_deformation_);

  int res = physics_->get_resolution();
  ::std::vector<unsigned char> data(res * res * 4);
  unsigned char* ptr = data.data();
  
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      double val = current_sand_(i, j);
      unsigned char c = static_cast<unsigned char>(::std::clamp(val * 255.0, 0.0, 255.0));
      *ptr++ = c; 
      *ptr++ = static_cast<unsigned char>(c * 0.9); 
      *ptr++ = static_cast<unsigned char>(c * 0.6); 
      *ptr++ = 255;
    }
  }
  glBindTexture(GL_TEXTURE_2D, plate_texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, res, res, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
  
  last_rendered_f = current_freq_;
  last_layout = ctx_.transducers;
}

void Application::render_pure_viewport(const nlohmann::json& symbol) {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2((float)width_, (float)height_));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus;
    
    if (ImGui::Begin("##PurePlot", nullptr, flags)) {
        if (ImPlot::BeginPlot("##PlatePlot", ImVec2(-1, -1), ImPlotFlags_Equal | ImPlotFlags_NoLegend | ImPlotFlags_NoMouseText)) {
            double x_min = -ctx_.lx / 2.0, x_max = ctx_.lx / 2.0;
            double y_min = -(ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0;
            double y_max = (ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0;
            
            ImPlot::SetupAxes(NULL, NULL, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
            ImPlot::SetupAxesLimits(x_min * 1.1, x_max * 1.1, y_min * 1.1, y_max * 1.1);
            ImPlot::PlotImage("Plate", (void*)(intptr_t)plate_texture_, ImPlotPoint(x_min, y_min), ImPlotPoint(x_max, y_max));
            ImPlot::EndPlot();
        }

        char caption[512];
        ::std::snprintf(caption, sizeof(caption), 
            "Symbol: %s\nFreq: %.1f Hz\nHW Gain: %.2f", 
            symbol.value("display_name", "UNKNOWN").c_str(),
            (double)current_freq_, ctx_.hardware_amp_gain);
        
        ImGui::GetWindowDrawList()->AddText(ImVec2(20, 20), IM_COL32(255, 255, 0, 255), caption);
    }
    ImGui::End();
}

void Application::run() {
  if (!init()) return;
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    
    if (is_batch_running_) {
        if (batch_current_idx_ < batch_data_.size()) {
            batch_progress_ = (float)batch_current_idx_ / batch_data_.size();
            const auto& symbol = batch_data_[batch_current_idx_];
            
            if (symbol.contains("hardware_config")) {
                const auto& hw_config = symbol["hardware_config"];
                ctx_.hardware_amp_gain = hw_config.value("hardware_amp_gain", 1.0);
                ctx_.transducer_max_power_w = hw_config.value("max_power_w", 25.0);
                
                ctx_.transducers.clear();
                double freq = 0.0;
                for (const auto& entry : hw_config["channels"]) {
                    Transducer t;
                    t.x = entry.value("x", 0.0);
                    t.y = entry.value("y", 0.0);
                    t.amplitude = entry.value("amplitude", 0.0);
                    t.phase_rad = entry.value("phase_deg", 0.0) * M_PI / 180.0;
                    freq = entry.value("frequency_hz", 0.0);
                    t.frequency = freq;
                    ctx_.transducers.push_back(t);
                }
                current_freq_ = static_cast<float>(freq);
                
                physics_->compute_visuals(static_cast<double>(current_freq_), ctx_, current_sand_, current_deformation_);

                int out_w = 1200;
                int out_h = static_cast<int>(1200.0 * (ctx_.ly / ctx_.lx));
                if (ctx_.geometry == Geometry::kCircular) out_h = 1200;
                
                int res = physics_->get_resolution();
                ::std::vector<unsigned char> pixels(out_w * out_h * 3, 0);

                for (int y = 0; y < out_h; ++y) {
                    for (int x = 0; x < out_w; ++x) {
                        float u = static_cast<float>(x) / (out_w - 1);
                        float v = static_cast<float>(y) / (out_h - 1);
                        int grid_x = ::std::clamp(static_cast<int>(u * res), 0, res - 1);
                        int grid_y = ::std::clamp(static_cast<int>(v * res), 0, res - 1);
                        double val = current_sand_(res - 1 - grid_y, grid_x); 
                        int idx = (y * out_w + x) * 3;
                        float intensity = ::std::clamp(static_cast<float>(val), 0.0f, 1.0f);
                        pixels[idx + 0] = static_cast<unsigned char>(intensity * 255.0f);
                        pixels[idx + 1] = static_cast<unsigned char>(intensity * 220.0f);
                        pixels[idx + 2] = static_cast<unsigned char>(intensity * 120.0f);
                    }
                }

                ::std::string json_image_path = symbol["ui_metadata"].value("image_path", "");
                ::std::string filename = (json_image_path.empty()) ? 
                    (batch_output_dir_ + "/CHLADNI_" + ::std::to_string((int)freq) + ".png") :
                    (".." + json_image_path.substr(1));

                ::std::filesystem::path p(filename);
                if (p.has_parent_path()) ::std::filesystem::create_directories(p.parent_path());
                stbi_write_png(filename.c_str(), out_w, out_h, 3, pixels.data(), out_w * 3);
                
                ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
                ImGui::Render();
                int dw, dh; glfwGetFramebufferSize(window_, &dw, &dh);
                glViewport(0, 0, dw, dh); glClearColor(0.05f, 0.05f, 0.05f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
                glfwSwapBuffers(window_);
                
                batch_current_idx_++;
            } else {
                batch_current_idx_++;
            }
        } else {
            is_batch_running_ = false;
            batch_status_ = "Batch Complete.";
            ::std::cout << "[Batch] All renders complete." << ::std::endl;
        }
        continue;
    }

    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
    if (reset_particles_requested_) { physics_->init_particles(num_particles_, ctx_.lx, ctx_.ly); reset_particles_requested_ = false; }
    if (is_sweeping_) {
        double current_time = glfwGetTime();
        if (current_time - last_sweep_tick_ > 0.05) { current_freq_ += sweep_step_; if (current_freq_ > sweep_end_) { current_freq_ = sweep_end_; is_sweeping_ = false; } last_sweep_tick_ = current_time; }
    }
    
    Eigen::MatrixXcd response = physics_->compute_driven_response(static_cast<double>(current_freq_), ctx_);
    if (show_particles_ && !analyzer_->is_analyzing()) {
        physics_->step_particles(response, ctx_.lx, ctx_.ly, 0.016);
    }
    
    update_texture();
    render_ui();
    
    ImGui::Render();
    int dw, dh; glfwGetFramebufferSize(window_, &dw, &dh);
    glViewport(0, 0, dw, dh); glClearColor(0.1f, 0.1f, 0.1f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
  }
}

void Application::render_ui() {
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
  
  render_panels(); 
  render_viewport();
  
  ImGui::Begin("Spectrum");
  static ::std::vector<double> spectrum_data; 
  static ::std::vector<double> freq_axis;
  static float last_f_max = 0.0f;
  
  if (last_f_max != f_max_limit_ || freq_axis.empty()) {
      freq_axis.clear();
      for (int i = 0; i < 500; ++i) freq_axis.push_back(20.0 + (double)i * (f_max_limit_ - 20.0) / 500.0);
      last_f_max = f_max_limit_;
  }

  static double last_update_time = -1.0; 
  double current_time = glfwGetTime();
  if (last_update_time < 0 || current_time - last_update_time > 0.1) { 
      spectrum_data = physics_->calculate_spectrum(20.0, f_max_limit_, 500, ctx_); 
      last_update_time = current_time; 
  }

  if (ImPlot::BeginPlot("##ResonanceSpectrum", ImVec2(-1, -1))) {
      ImPlot::SetupAxes("Frequency (Hz)", "dB"); 
      ImPlot::SetupAxesLimits(20, f_max_limit_, -100, 50);
      if (!spectrum_data.empty() && !freq_axis.empty()) {
          ImPlot::PlotLine("Energy", freq_axis.data(), spectrum_data.data(), 500);
      }
      double cur_f = (double)current_freq_; 
      if (ImPlot::DragLineX(0, &cur_f, ImVec4(1,0,0,1))) current_freq_ = (float)cur_f;
      ImPlot::EndPlot();
  }
  ImGui::End();

  render_sweeper_tab();
}

void Application::render_viewport() {
  ImGui::Begin("Engineering Workstation");
  
  if (ImGui::BeginTabBar("WorkstationTabs")) {
      
      if (ImGui::BeginTabItem("Live Plate Viewport")) {
          ImGui::Checkbox("Show Particles", &show_particles_);
          if (analyzer_->is_analyzing()) {
              ImGui::SameLine();
              ImGui::TextColored(ImVec4(1,0,0,1), " (Paused: CPU optimizing background tasks)");
          }

          if (ImPlot::BeginPlot("##PlatePlot", ImVec2(-1, -1), ImPlotFlags_Equal | ImPlotFlags_NoLegend)) {
              double x_min = -ctx_.lx / 2.0, x_max = ctx_.lx / 2.0;
              double y_min = -(ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0;
              double y_max = (ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0;
              
              ImPlot::SetupAxes(NULL, NULL, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
              ImPlot::SetupAxesLimits(x_min * 1.1, x_max * 1.1, y_min * 1.1, y_max * 1.1);
              
              ImPlot::PlotImage("Plate", (void*)(intptr_t)plate_texture_, ImPlotPoint(x_min, y_min), ImPlotPoint(x_max, y_max));
              
              for (size_t i = 0; i < ctx_.transducers.size(); ++i) {
                  ::std::string id = "T" + ::std::to_string(i + 1);
                  if (ImPlot::DragPoint(static_cast<int>(i), &ctx_.transducers[i].x, &ctx_.transducers[i].y, ImVec4(1, 0.5, 0, 1), 4)) {
                      physics_->clamp_transducer(ctx_.transducers[i], ctx_);
                  }
                  
                  static double circle_x[64], circle_y[64];
                  static bool circle_init = false;
                  if (!circle_init) {
                      for (int j = 0; j < 64; ++j) {
                          double a = 2.0 * 3.14159 * j / 63.0;
                          circle_x[j] = ::std::cos(a);
                          circle_y[j] = ::std::sin(a);
                      }
                      circle_init = true;
                  }
                  double draw_x[64], draw_y[64];
                  for (int j = 0; j < 64; ++j) {
                      draw_x[j] = ctx_.transducers[i].x + ctx_.transducer_radius_m * circle_x[j];
                      draw_y[j] = ctx_.transducers[i].y + ctx_.transducer_radius_m * circle_y[j];
                  }

                  ImPlot::PlotLine("Clearance", draw_x, draw_y, 64);
                  ImPlot::Annotation(ctx_.transducers[i].x, ctx_.transducers[i].y, ImVec4(0,0,0,0), ImVec2(10, -10), true, "%s", id.c_str());
              }
              
              if (show_particles_ && !analyzer_->is_analyzing()) {
                  const auto& pts = physics_->get_particles();
                  if (pts.rows() > 0) {
                      ImPlotSpec spec; spec.Marker = ImPlotMarker_Circle; spec.MarkerSize = 1.0f; spec.MarkerFillColor = ImVec4(1, 1, 0.8, 1); spec.MarkerLineColor = ImVec4(1, 1, 0.8, 1);
                      ImPlot::PlotScatter("Particles", pts.col(0).data(), pts.col(1).data(), static_cast<int>(pts.rows()), spec);
                  }
              }
              ImPlot::EndPlot();
          }
          ImGui::EndTabItem();
      }

      if (ImGui::BeginTabItem("Symmetry Heatmap Tool")) {
          ImGui::Text("Load previous simulation data to analyze the optimization landscape:");
          ImGui::SameLine();
          if (ImGui::Button("Refresh")) refresh_sim_files();

          const HeatmapData& hm = analyzer_->get_heatmap_data();

          if (!available_sim_files_.empty()) {
              const char* current_file = available_sim_files_[selected_sim_file_idx_].c_str();
              if (ImGui::BeginCombo("##FileCombo", current_file)) {
                  for (size_t i = 0; i < available_sim_files_.size(); ++i) {
                      bool is_selected = (selected_sim_file_idx_ == static_cast<int>(i));
                      if (ImGui::Selectable(available_sim_files_[i].c_str(), is_selected)) selected_sim_file_idx_ = static_cast<int>(i);
                      if (is_selected) ImGui::SetItemDefaultFocus();
                  }
                  ImGui::EndCombo();
              }
              
              ImGui::SameLine();
              if (ImGui::Button("Load File Matrix")) {
                  if (analyzer_->load_sim_results("../sim/transducer_analysis/" + available_sim_files_[selected_sim_file_idx_], ctx_)) {
                      stage2_params_.use_roi = false;
                      stage2_params_.step_size_m = 0.015f; 
                  }
              }
              
              if (hm.valid) {
                  ImGui::SameLine();
                  if (ImGui::Button("Clear Heatmap Memory")) {
                      analyzer_->clear_heatmap();
                      stage2_params_.use_roi = false;
                      stage2_params_.step_size_m = 0.015f; 
                  }
              }
          } else {
              ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "No .json files found in ../sim/transducer_analysis/");
              if (hm.valid) {
                  ImGui::SameLine();
                  if (ImGui::Button("Clear Heatmap Memory")) {
                      analyzer_->clear_heatmap();
                      stage2_params_.use_roi = false;
                      stage2_params_.step_size_m = 0.015f; 
                  }
              }
          }

          ImGui::Separator();

          if (hm.valid) {
              ImGui::TextColored(ImVec4(0, 1, 1, 1), "Tip: Drag a selection box (Shift+LeftClick) OR Zoom in (RightClick+Drag), then apply it below.");
              
              ::std::vector<float> unique_scores;
              for (float s : hm.scores) {
                  if (s >= 0.0f && ::std::find(unique_scores.begin(), unique_scores.end(), s) == unique_scores.end()) {
                      unique_scores.push_back(s);
                  }
              }
              ::std::sort(unique_scores.begin(), unique_scores.end());
              
              ::std::vector<float> mapped_scores(hm.scores.size(), 0.0f);
              for (size_t i = 0; i < hm.scores.size(); ++i) {
                  if (hm.scores[i] >= 0.0f) {
                      auto it = ::std::find(unique_scores.begin(), unique_scores.end(), hm.scores[i]);
                      mapped_scores[i] = static_cast<float>(::std::distance(unique_scores.begin(), it) + 1);
                  }
              }
              float max_bin = ::std::max(1.0f, static_cast<float>(unique_scores.size()));

              float step_x = (hm.nx > 1) ? (hm.dx_max - hm.dx_min) / (hm.nx - 1) : 0.015f;
              float step_y = (hm.ny > 1) ? (hm.dy_max - hm.dy_min) / (hm.ny - 1) : 0.015f;

              ::std::vector<float> display_scores(hm.nx * hm.ny, 0.0f);
              for (int y = 0; y < hm.ny; ++y) {
                  for (int x = 0; x < hm.nx; ++x) {
                      int inverted_y = hm.ny - 1 - y;
                      display_scores[inverted_y * hm.nx + x] = mapped_scores[y * hm.nx + x];
                  }
              }

              static bool cancel_roi = false;
              bool has_selection = false;
              ImPlotRect selection;
              ImPlotRect limits;
              bool has_limits = false;

              if (ImPlot::BeginPlot("##SymmetryMap", ImVec2(-1, -60), ImPlotFlags_NoLegend)) {
                  ImPlot::SetupAxes("Transducer dx Offset (m)", "Transducer dy Offset (m)");
                  ImPlot::SetupAxesLimits(0, ctx_.lx / 2.0, 0, (ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0);
                  
                  limits = ImPlot::GetPlotLimits();
                  has_limits = true;

                  ImPlot::PlotHeatmap("Alphabet Size", display_scores.data(), hm.ny, hm.nx, 0.0f, max_bin, nullptr,
                                      ImPlotPoint((double)(hm.dx_min - step_x/2.0f), (double)(hm.dy_min - step_y/2.0f)), 
                                      ImPlotPoint((double)(hm.dx_max + step_x/2.0f), (double)(hm.dy_max + step_y/2.0f)));
                  
                  for (int y = 0; y < hm.ny; ++y) {
                      for (int x = 0; x < hm.nx; ++x) {
                          float val = hm.scores[y * hm.nx + x];
                          float px = hm.dx_min + x * step_x;
                          float py = hm.dy_min + y * step_y;
                          char buf[16]; ::std::snprintf(buf, sizeof(buf), "%.0f", val);
                          ImPlot::PlotText(buf, px, py);
                      }
                  }

                  const auto& tops = analyzer_->get_top_layouts();
                  if (!tops.empty()) {
                      for (size_t i = 0; i < tops.size(); ++i) {
                          double tx = static_cast<double>(tops[i].grid_dx);
                          double ty = static_cast<double>(tops[i].grid_dy);
                          char lbl[32]; ::std::snprintf(lbl, sizeof(lbl), "  #%zu  ", i+1);
                          ImPlot::Annotation(tx, ty, ImVec4(0.9f, 0.7f, 0.0f, 1.0f), ImVec2(10, 10), true, "%s", lbl);
                      }
                  }

                  if (cancel_roi) {
                      ImPlot::CancelPlotSelection();
                      cancel_roi = false;
                  }

                  if (ImPlot::IsPlotSelected()) {
                      has_selection = true;
                      selection = ImPlot::GetPlotSelection();
                  }

                  ImPlot::EndPlot();
              }

              if (has_selection) {
                  ImGui::TextColored(ImVec4(1, 1, 0, 1), "Selection Box: X [%.3f, %.3f]  |  Y [%.3f, %.3f]", 
                      selection.X.Min, selection.X.Max, selection.Y.Min, selection.Y.Max);
                      
                  ImGui::SameLine();
                  if (ImGui::Button("Apply Selection as ROI", ImVec2(-1, 0))) {
                      stage2_params_.use_roi = true;
                      stage2_params_.roi_dx_min = ::std::max(0.0f, static_cast<float>(selection.X.Min));
                      stage2_params_.roi_dx_max = ::std::min(static_cast<float>(ctx_.lx / 2.0), static_cast<float>(selection.X.Max));
                      stage2_params_.roi_dy_min = ::std::max(0.0f, static_cast<float>(selection.Y.Min));
                      stage2_params_.roi_dy_max = ::std::min(static_cast<float>((ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0), static_cast<float>(selection.Y.Max));
                      stage2_params_.step_size_m = 0.001f; 
                      cancel_roi = true; 
                  }
              } else if (has_limits) {
                  ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "Current Viewport: X [%.3f, %.3f]  |  Y [%.3f, %.3f]", 
                      limits.X.Min, limits.X.Max, limits.Y.Min, limits.Y.Max);
                      
                  ImGui::SameLine();
                  if (ImGui::Button("Capture Current View as ROI", ImVec2(-1, 0))) {
                      stage2_params_.use_roi = true;
                      stage2_params_.roi_dx_min = ::std::max(0.0f, static_cast<float>(limits.X.Min));
                      stage2_params_.roi_dx_max = ::std::min(static_cast<float>(ctx_.lx / 2.0), static_cast<float>(limits.X.Max));
                      stage2_params_.roi_dy_min = ::std::max(0.0f, static_cast<float>(limits.Y.Min));
                      stage2_params_.roi_dy_max = ::std::min(static_cast<float>((ctx_.geometry == Geometry::kCircular ? ctx_.lx : ctx_.ly) / 2.0), static_cast<float>(limits.Y.Max));
                      stage2_params_.step_size_m = 0.001f; 
                  }
              }

          } else {
              ImGui::Text("Waiting for matrix data... Run Stage 2 or Load a File to view the Heatmap.");
          }
          ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
  }
  ImGui::End();
}

void Application::render_sweeper_tab() {
    ImGui::Begin("Frequency Sweeper");
    ImGui::InputFloat("Start Frequency", &sweep_start_); ImGui::InputFloat("End Frequency", &sweep_end_); ImGui::InputFloat("Step (Hz)", &sweep_step_);
    if (is_sweeping_) { if (ImGui::Button("Stop Sweep")) is_sweeping_ = false; }
    else { if (ImGui::Button("Start Sweep")) { current_freq_ = sweep_start_; is_sweeping_ = true; last_sweep_tick_ = glfwGetTime(); } }
    ImGui::End();
}

void Application::render_panels() {
  Panels::draw_main_ui(ctx_, this, *analyzer_, current_freq_, f_max_limit_, is_batch_running_, batch_progress_, batch_status_);
}

void Application::snap_to_resonance(int direction) {
    ::std::vector<double> freqs = physics_->get_resonant_frequencies(ctx_);
    if (freqs.empty()) return;
    double current = static_cast<double>(current_freq_); double target = current; bool found = false;
    if (direction > 0) { for (double f : freqs) { if (f > current + 1.0) { target = f; found = true; break; } } if (!found) target = freqs.front(); }
    else { for (auto it = freqs.rbegin(); it != freqs.rend(); ++it) { if (*it < current - 1.0) { target = *it; found = true; break; } } if (!found) target = freqs.back(); }
    current_freq_ = static_cast<float>(target);
}

void Application::apply_preset(const ::std::string& name) {
    ctx_.transducers.clear();
    if (name == "1-center") {
        ctx_.transducers.push_back({0.0, 0.0, 1.0, 0.0, ::std::nullopt});
    } else if (name == "4-corners") {
        double dx = 0.45 * ctx_.lx;
        double dy = 0.45 * ctx_.ly;
        ctx_.transducers.push_back({dx, dy, 1.0, 0.0, ::std::nullopt});
        ctx_.transducers.push_back({-dx, dy, 1.0, 3.14159, ::std::nullopt});
        ctx_.transducers.push_back({-dx, -dy, 1.0, 0.0, ::std::nullopt});
        ctx_.transducers.push_back({dx, -dy, 1.0, 3.14159, ::std::nullopt});
    }
}

void Application::save_screenshot(const ::std::string& filename) {
    ::std::vector<unsigned char> pixels(width_ * height_ * 3);
    glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    ::std::vector<unsigned char> flipped(width_ * height_ * 3);
    for (int y = 0; y < height_; y++) { ::std::memcpy(&flipped[y * width_ * 3], &pixels[(height_ - 1 - y) * width_ * 3], width_ * 3); }
    stbi_write_png(filename.c_str(), width_, height_, 3, flipped.data(), width_ * 3);
}

void Application::start_batch_plotting(const ::std::string& json_path, const ::std::string& output_dir) {
    if (is_batch_running_) return;
    
    ::std::ifstream file(json_path);
    if (!file.is_open()) { 
        ::std::cerr << "[Batch] Error: JSON not found at " << json_path << ::std::endl;
        batch_status_ = "Error: JSON not found."; 
        return; 
    }
    
    try {
        file >> batch_data_;
    } catch (...) {
        batch_status_ = "Error: JSON parse failed.";
        return;
    }

    if (!batch_data_.is_array()) { 
        batch_status_ = "Error: JSON is not an array."; 
        return; 
    }

    ::std::cout << "[Batch] Queued render of " << batch_data_.size() << " symbols." << ::std::endl;
    is_batch_running_ = true;
    batch_current_idx_ = 0;
    batch_output_dir_ = output_dir;
}

} // namespace chladni