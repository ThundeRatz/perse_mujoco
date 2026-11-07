#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
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
  MujocoViewer(const std::string &model_path,
               const std::string &sim_socket_path = "/tmp/perse_mujoco.sock",
               const std::string &viewer_socket_path = "/tmp/perse_viewer.sock");
  ~MujocoViewer();

  bool init();
  void run();
  void stop();

  // Controle de câmera, visualização e captura (para MCP)
  void setCamera(double lookat[3], double distance, double azimuth, double elevation);
  bool focusOnBody(const std::string &body_name, double distance = 2.0);
  bool setVisualizationOption(const std::string &option, bool enabled);
  bool captureFrame(const std::string &output_png_path, int width = 640, int height = 480);

  // Acessores para callbacks GLFW
  mjModel *getModel() { return m_; }
  mjData *getData() { return d_; }
  mjvCamera *getCamera() { return &cam_; }
  mjvOption *getOption() { return &opt_; }
  mjvScene *getScene() { return &scn_; }
  mjrContext *getContext() { return &con_; }
  mjvPerturb *getPerturb() { return &pert_; }
  std::mutex &getDataMutex() { return data_mutex_; }

  void sendSimCommand(const json &cmd);
  void applySpringForce();

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

  std::mutex data_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_to_sim_{false};

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
