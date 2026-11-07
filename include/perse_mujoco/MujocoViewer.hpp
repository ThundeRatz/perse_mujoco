#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace perse_mujoco {

using json = nlohmann::json;

class MujocoViewer {
public:
  MujocoViewer(
      const std::string &model_path,
      const std::string &sim_socket_path = "/tmp/perse_mujoco.sock",
      const std::string &viewer_socket_path = "/tmp/perse_viewer.sock");
  ~MujocoViewer();

  bool init();
  void run();
  void stop();

  // Controle de câmera, visualização e captura (para MCP)
  void setCamera(double lookat[3], double distance, double azimuth,
                 double elevation);
  bool focusOnBody(const std::string &body_name, double distance = 2.0);
  bool setVisualizationOption(const std::string &option, bool enabled);
  bool captureFrame(const std::string &output_png_path, int width = 640,
                    int height = 480);

  // Acessores para callbacks GLFW
  mjModel *getModel() { return m_; }
  mjData *getData() { return d_; }
  mjvCamera *getCamera() { return &cam_; }
  mjvOption *getOption() { return &opt_; }
  mjvScene *getScene() { return &scn_; }
  mjrContext *getContext() { return &con_; }
  mjvPerturb *getPerturb() { return &pert_; }
  mjUI *getUI() { return &ui_; }
  mjuiState *getUIState() { return &uistate_; }
  std::mutex &getDataMutex() { return data_mutex_; }

  void sendSimCommand(const json &cmd);
  void applySpringForce();
  void onUiEvent(mjuiItem *it);

  // Controle de rastreamento do robô (Follow camera)
  void toggleTracking();
  bool isTracking() const { return tracking_robot_; }
  void setTracking(bool tracking);
  void resetSimulation();

private:
  void ipcSyncLoop();
  void controlServerLoop();
  void handleControlClient(int client_fd);
  json dispatchControlCommand(const json &req);

  std::string model_path_;
  std::string sim_socket_path_;
  std::string viewer_socket_path_;

  mjModel *m_{nullptr};
  mjData *d_{nullptr};

  // Estruturas oficiais de visualização e perturbação do MuJoCo
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  mjvPerturb pert_;
  GLFWwindow *window_{nullptr};

  // Sistema de interface nativo mjUI
  mjUI ui_;
  mjuiState uistate_;
  double speed_slider_val_{1.0};
  double last_sent_speed_{1.0};

  // Métricas de desempenho para overlay
  std::atomic<double> sim_physics_fps_{0.0};
  std::atomic<double> sim_speed_multiplier_{0.9};
  double render_fps_{0.0};

  std::mutex data_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_to_sim_{false};
  std::atomic<bool> tracking_robot_{false};
  int tracking_body_id_{-1};

  std::thread sync_thread_;
  std::thread control_thread_;
  int control_server_fd_{-1};

  // Sincronização de captura de frames para a thread principal GLFW/OpenGL
  struct ScreenshotRequest {
    bool pending{false};
    std::string path;
    int width{0};
    int height{0};
    bool success{false};
    std::mutex mtx;
    std::condition_variable cv;
  } screenshot_req_;
};

} // namespace perse_mujoco
