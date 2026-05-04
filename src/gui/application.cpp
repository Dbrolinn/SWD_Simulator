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

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Vertex Shader for 3D Surface
const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in float aHeight;
    out float Height;
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * view * model * vec4(aPos.x, aHeight * 0.1, aPos.z, 1.0);
        Height = aHeight;
    }
)";

// Fragment Shader for 3D Surface
const char* fragmentShaderSource = R"(
    #version 330 core
    in float Height;
    out vec4 FragColor;
    void main() {
        vec3 color = mix(vec3(0.2, 0.4, 0.8), vec3(0.8, 0.2, 0.2), (Height + 1.0) * 0.5);
        FragColor = vec4(color, 1.0);
    }
)";

namespace chladni {

Application::Application(const ::std::string& title, int width, int height)
    : window_(nullptr), title_(title), width_(width), height_(height) {
  physics_ = ::std::make_shared<PhysicsEngine>(400);
  analyzer_ = ::std::make_shared<Analyzer>(physics_);

  // Default context (SWAID Hardware Defaults)
  ctx_.geometry = Geometry::kRectangular;
  ctx_.lx = 0.30;
  ctx_.ly = 0.20;
  ctx_.h = 0.0010;
  ctx_.e = 193e9; // Steel
  ctx_.rho = 8000.0;
  ctx_.nu = 0.29;
  ctx_.damping = 0.005;
  ctx_.n_modes = 20;
  ctx_.sign = 1;
  ctx_.base_volume_1 = 1.0;
  ctx_.base_volume_2 = 1.0;
  ctx_.calib_m = 1.0;
  ctx_.calib_b = 0.0;
  ctx_.speaker = {};
  
  // Default to exactly 4 transducers
  ctx_.transducers.clear();
  for (int i = 0; i < 4; ++i) {
    ctx_.transducers.push_back({0.0, 0.0, 1.0, 0.0, ::std::nullopt});
  }
}

Application::~Application() {
  shutdown();
}

bool Application::init() {
  if (!glfwInit()) return false;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    return false;
  }

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

  init_3d_resources();

  return true;
}

void Application::init_3d_resources() {
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    shader_program_ = glCreateProgram();
    glAttachShader(shader_program_, vertexShader);
    glAttachShader(shader_program_, fragmentShader);
    glLinkProgram(shader_program_);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    for (int i = 0; i <= mesh_res_; ++i) {
        for (int j = 0; j <= mesh_res_; ++j) {
            mesh_vertices_.push_back((float)j / mesh_res_ - 0.5f);
            mesh_vertices_.push_back(0.0f);
            mesh_vertices_.push_back((float)i / mesh_res_ - 0.5f);
            mesh_vertices_.push_back(0.0f);
        }
    }

    for (int i = 0; i < mesh_res_; ++i) {
        for (int j = 0; j < mesh_res_; ++j) {
            int row1 = i * (mesh_res_ + 1);
            int row2 = (i + 1) * (mesh_res_ + 1);
            mesh_indices_.push_back(row1 + j);
            mesh_indices_.push_back(row1 + j + 1);
            mesh_indices_.push_back(row2 + j);
            mesh_indices_.push_back(row1 + j + 1);
            mesh_indices_.push_back(row2 + j + 1);
            mesh_indices_.push_back(row2 + j);
        }
    }

    glGenVertexArrays(1, &surface_vao_);
    glGenBuffers(1, &surface_vbo_);
    glGenBuffers(1, &surface_ebo_);

    glBindVertexArray(surface_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, surface_vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh_vertices_.size() * sizeof(float)), mesh_vertices_.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, surface_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh_indices_.size() * sizeof(unsigned int)), mesh_indices_.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    glGenTextures(1, &f_texture_);
    glBindTexture(GL_TEXTURE_2D, f_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 800, 600, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, f_texture_, 0);

    glGenRenderbuffers(1, &rbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 800, 600);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo_);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Application::update_3d_mesh() {
    for (int i = 0; i <= mesh_res_; ++i) {
        for (int j = 0; j <= mesh_res_; ++j) {
            int di = ::std::clamp((int)(i * 200 / mesh_res_), 0, 199);
            int dj = ::std::clamp((int)(j * 200 / mesh_res_), 0, 199);
            float h = (float)current_deformation_(di, dj);
            mesh_vertices_[(i * (mesh_res_ + 1) + j) * 4 + 3] = h;
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, surface_vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(mesh_vertices_.size() * sizeof(float)), mesh_vertices_.data());
}

void Application::draw_3d_mesh() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, 800, 600);
    glClearColor(0.15f, 0.15f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glUseProgram(shader_program_);

    float time = (float)glfwGetTime();
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float angle = time * 0.5f;
    model(0,0) = ::std::cos(angle); model(0,2) = ::std::sin(angle);
    model(2,0) = -::std::sin(angle); model(2,2) = ::std::cos(angle);

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view(2,3) = -1.5f;

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    float aspect = 800.0f / 600.0f;
    float fov = 45.0f * 3.14159f / 180.0f;
    float f = 1.0f / ::std::tan(fov / 2.0f);
    proj(0,0) = f / aspect;
    proj(1,1) = f;
    proj(2,2) = (100.0f + 0.1f) / (0.1f - 100.0f);
    proj(2,3) = (2.0f * 100.0f * 0.1f) / (0.1f - 100.0f);
    proj(3,2) = -1.0f;

    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, model.data());
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, view.data());
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, proj.data());

    glBindVertexArray(surface_vao_);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, (int)mesh_indices_.size(), GL_UNSIGNED_INT, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Application::shutdown() {
  if (plate_texture_) glDeleteTextures(1, &plate_texture_);
  if (f_texture_) glDeleteTextures(1, &f_texture_);
  if (fbo_) glDeleteFramebuffers(1, &fbo_);
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
                  ::std::abs(ctx_.transducers[i].y - last_layout[i].y) > 1e-6) {
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

        // Burn metadata onto the clean image
        char caption[512];
        ::std::snprintf(caption, sizeof(caption), 
            "Symbol: %s\nFreq: %.1f Hz\nAmp1: %.2f | Amp2: %.2f", 
            symbol.value("display_name", "UNKNOWN").c_str(),
            (double)current_freq_, ctx_.base_volume_1, ctx_.base_volume_2);
        
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
                ctx_.base_volume_1 = hw_config.value("base_volume_1", 1.0);
                ctx_.base_volume_2 = hw_config.value("base_volume_2", 1.0);
                
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
                update_texture();

                // PURE RENDER (No UI Panels, No Particles)
                ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
                
                render_pure_viewport(symbol);

                ImGui::Render();
                
                int dw, dh; glfwGetFramebufferSize(window_, &dw, &dh);
                glViewport(0, 0, dw, dh); glClearColor(0.05f, 0.05f, 0.05f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

                // Save
                ::std::string json_image_path = symbol["ui_metadata"].value("image_path", "");
                ::std::string filename = (json_image_path.empty()) ? 
                    (batch_output_dir_ + "/CHLADNI_" + ::std::to_string((int)freq) + ".png") :
                    (".." + json_image_path.substr(1));

                ::std::filesystem::path p(filename);
                if (p.has_parent_path()) ::std::filesystem::create_directories(p.parent_path());
                save_screenshot(filename);
                
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
        continue; // Skip normal UI loop during batch
    }

    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
    if (reset_particles_requested_) { physics_->init_particles(num_particles_, ctx_.lx, ctx_.ly); reset_particles_requested_ = false; }
    if (is_sweeping_) {
        double current_time = glfwGetTime();
        if (current_time - last_sweep_tick_ > 0.05) { current_freq_ += sweep_step_; if (current_freq_ > sweep_end_) { current_freq_ = sweep_end_; is_sweeping_ = false; } last_sweep_tick_ = current_time; }
    }
    Eigen::MatrixXcd response = physics_->compute_driven_response(static_cast<double>(current_freq_), ctx_);
    if (show_particles_) physics_->step_particles(response, ctx_.lx, ctx_.ly, 0.016);
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
  render_panels(); render_viewport();
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
  ImGui::Begin("Plate Viewport");
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
          // Draw mechanical clearance boundary (50mm radius)
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
              draw_x[j] = ctx_.transducers[i].x + 0.025 * circle_x[j];
              draw_y[j] = ctx_.transducers[i].y + 0.025 * circle_y[j];
          }

          ImPlot::PlotLine("Clearance", draw_x, draw_y, 64);
          ImPlot::Annotation(ctx_.transducers[i].x, ctx_.transducers[i].y, ImVec4(0,0,0,0), ImVec2(10, -10), true, "%s", id.c_str());
      }
      if (show_particles_) {
          const auto& pts = physics_->get_particles();
          if (pts.rows() > 0) {
              ImPlotSpec spec; spec.Marker = ImPlotMarker_Circle; spec.MarkerSize = 1.0f; spec.MarkerFillColor = ImVec4(1, 1, 0.8, 1); spec.MarkerLineColor = ImVec4(1, 1, 0.8, 1);
              ImPlot::PlotScatter("Particles", pts.col(0).data(), pts.col(1).data(), static_cast<int>(pts.rows()), spec);
          }
      }
      ImPlot::EndPlot();
  }
  ImGui::End();
}

void Application::render_3d_viewport() {
    ImGui::Begin("3D Deformation");
    ImVec2 size = ImGui::GetContentRegionAvail();
    ImGui::Image((void*)(intptr_t)f_texture_, size, ImVec2(0, 1), ImVec2(1, 0));
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

