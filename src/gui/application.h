/**
 * @file application.h
 * @brief Main application window and ImGui context management.
 */

#pragma once

#include <string>
#include <memory>
#include <future>
#include <atomic>
#include <vector>
#include "core/physics.h"
#include "core/analyzer.h"

// Forward declarations
struct GLFWwindow;

namespace chladni {

/**
 * @class Application
 * @brief Manages the GLFW window and ImGui rendering loop.
 */
class Application {
 public:
  Application(const ::std::string& title, int width, int height);
  ~Application();

  /**
   * @brief Runs the main application loop.
   */
  void run();

  // Public API for Panels
  void snap_to_resonance(int direction);
  void apply_preset(const ::std::string& name);
  void save_screenshot(const ::std::string& filename);
  void start_batch_plotting(const ::std::string& csv_path, const ::std::string& output_dir);

  ::std::shared_ptr<PhysicsEngine> get_physics() { return physics_; }

 private:
  bool init();
  void shutdown();

  void render_ui();
  void render_viewport();
  void render_pure_viewport(const nlohmann::json& symbol);
  void render_panels();
  void render_3d_viewport();
  void render_sweeper_tab();

  void update_texture();
  
  // Batch State
  bool is_batch_running_ = false;
  float batch_progress_ = 0.0f;
  ::std::string batch_status_ = "Idle";
  nlohmann::json batch_data_;
  size_t batch_current_idx_ = 0;
  ::std::string batch_output_dir_;
  
  GLFWwindow* window_;
  ::std::string title_;
  int width_, height_;

  unsigned int plate_texture_ = 0;
  Eigen::MatrixXd current_sand_;
  Eigen::MatrixXd current_deformation_;

  ::std::shared_ptr<PhysicsEngine> physics_;
  ::std::shared_ptr<Analyzer> analyzer_;
  SimulationContext ctx_;

  // UI State
  float current_freq_ = 100.0f;
  float f_max_limit_ = 10000.0f;
  bool show_particles_ = true;
  int num_particles_ = 2000;
  bool reset_particles_requested_ = true;

  // Analyzer State
  ::std::future<::std::vector<LayoutResult>> analyzer_future_;
  bool is_analyzing_ = false;
  ::std::vector<LayoutResult> analyzer_results_;

  // Sweeper State
  bool is_sweeping_ = false;
  float sweep_start_ = 100.0f;
  float sweep_end_ = 5000.0f;
  float sweep_step_ = 10.0f;
  double last_sweep_tick_ = 0;

  // 3D Rendering (OpenGL)
  unsigned int surface_vao_ = 0, surface_vbo_ = 0, surface_ebo_ = 0;
  unsigned int shader_program_ = 0;
  unsigned int fbo_ = 0, f_texture_ = 0, rbo_ = 0;
  int mesh_res_ = 100; // Lower res for 3D mesh performance
  ::std::vector<float> mesh_vertices_;
  ::std::vector<unsigned int> mesh_indices_;

  void init_3d_resources();
  void update_3d_mesh();
  void draw_3d_mesh();
};

} // namespace chladni
