#pragma once

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace perse_mujoco {

using json = nlohmann::json;

class MujocoIpcBridge {
public:
  enum class SimMode { FREE_RUN, STEP_MODE };

  MujocoIpcBridge(mjModel *m, mjData *d, std::mutex &sim_mutex,
                  const std::string &socket_path = "/tmp/perse_mujoco.sock");
  ~MujocoIpcBridge();

  bool start();
  void stop();

  SimMode getMode() const;
  void setMode(SimMode mode);

  bool shouldStep(int &steps_to_advance);

  double getSpeedMultiplier() const;
  void setSpeedMultiplier(double mult);

  void updatePhysicsFps(double fps);

  // Broadcast do estado atual da simulação para visualizadores conectados
  void broadcastState();

  // Callback chamado quando um reset é disparado via IPC
  void setResetCallback(std::function<void()> callback);

private:
  void serverLoop();
  void handleClient(int client_fd);
  json dispatchCommand(const json &req, int client_fd);

  std::function<void()> reset_callback_{nullptr};

  mjModel *m_{nullptr};
  mjData *d_{nullptr};
  std::mutex &sim_mutex_;
  std::string socket_path_;

  std::atomic<bool> running_{false};
  std::atomic<bool> paused_{false};
  std::atomic<SimMode> mode_{SimMode::FREE_RUN};
  std::atomic<double> speed_multiplier_{1.0};
  std::atomic<double> physics_fps_{0.0};

  std::mutex step_mutex_;
  int pending_steps_{0};

  // Clientes de visualização (subscribers contínuos de estado)
  std::mutex subscribers_mutex_;
  std::vector<int> subscribers_;

  int server_fd_{-1};
  std::thread server_thread_;
};

} // namespace perse_mujoco
