#include "perse_mujoco/MujocoIpcBridge.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace perse_mujoco {

MujocoIpcBridge::MujocoIpcBridge(mjModel *m, mjData *d, std::mutex &sim_mutex,
                                 const std::string &socket_path)
    : m_(m), d_(d), sim_mutex_(sim_mutex), socket_path_(socket_path) {}

MujocoIpcBridge::~MujocoIpcBridge() { stop(); }

bool MujocoIpcBridge::start() {
  running_ = true;
  server_thread_ = std::thread(&MujocoIpcBridge::serverLoop, this);
  return true;
}

void MujocoIpcBridge::stop() {
  if (running_) {
    running_ = false;
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
      server_fd_ = -1;
    }
    {
      std::lock_guard<std::mutex> lock(subscribers_mutex_);
      for (int fd : subscribers_) {
        shutdown(fd, SHUT_RDWR);
        close(fd);
      }
      subscribers_.clear();
    }
    unlink(socket_path_.c_str());
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }
}

MujocoIpcBridge::SimMode MujocoIpcBridge::getMode() const { return mode_; }

void MujocoIpcBridge::setMode(SimMode mode) { mode_ = mode; }

double MujocoIpcBridge::getSpeedMultiplier() const { return speed_multiplier_.load(); }

void MujocoIpcBridge::setSpeedMultiplier(double mult) {
  speed_multiplier_.store(std::max(0.01, std::min(mult, 10.0)));
}

void MujocoIpcBridge::updatePhysicsFps(double fps) {
  physics_fps_.store(fps);
}

void MujocoIpcBridge::setResetCallback(std::function<void()> callback) {
  reset_callback_ = callback;
}

bool MujocoIpcBridge::shouldStep(int &steps_to_advance) {
  if (mode_ == SimMode::FREE_RUN) {
    if (paused_) {
      steps_to_advance = 0;
      return false;
    }
    return true;
  } else {
    // STEP_MODE
    std::lock_guard<std::mutex> lock(step_mutex_);
    if (pending_steps_ > 0) {
      steps_to_advance = pending_steps_;
      pending_steps_ = 0;
      return true;
    }
    steps_to_advance = 0;
    return false;
  }
}

void MujocoIpcBridge::broadcastState() {
  std::vector<int> current_subscribers;
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    if (subscribers_.empty()) return;
    current_subscribers = subscribers_;
  }

  json state_msg;
  {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    std::vector<double> qpos(d_->qpos, d_->qpos + m_->nq);
    std::vector<double> qvel(d_->qvel, d_->qvel + m_->nv);
    state_msg = {
        {"type", "state_update"},
        {"sim_time", d_->time},
        {"paused", paused_.load()},
        {"speed_multiplier", speed_multiplier_.load()},
        {"physics_fps", physics_fps_.load()},
        {"qpos", qpos},
        {"qvel", qvel}
    };
  }

  std::string msg_str = state_msg.dump() + "\n";
  std::vector<int> dead_subscribers;

  for (int fd : current_subscribers) {
    ssize_t bytes_written = send(fd, msg_str.c_str(), msg_str.size(), MSG_NOSIGNAL);
    if (bytes_written < 0) {
      dead_subscribers.push_back(fd);
    }
  }

  if (!dead_subscribers.empty()) {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    for (int dead_fd : dead_subscribers) {
      close(dead_fd);
      subscribers_.erase(std::remove(subscribers_.begin(), subscribers_.end(), dead_fd), subscribers_.end());
    }
  }
}

void MujocoIpcBridge::serverLoop() {
  unlink(socket_path_.c_str());

  server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    perror("socket");
    return;
  }

  struct sockaddr_un addr {};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (bind(server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(server_fd_);
    server_fd_ = -1;
    return;
  }

  if (listen(server_fd_, 10) < 0) {
    perror("listen");
    close(server_fd_);
    server_fd_ = -1;
    return;
  }

  while (running_) {
    int client_fd = accept(server_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (!running_) break;
      continue;
    }

    // Lança uma thread para tratar requisições do cliente
    std::thread(&MujocoIpcBridge::handleClient, this, client_fd).detach();
  }
}

void MujocoIpcBridge::handleClient(int client_fd) {
  std::string buffer;
  char chunk[4096];

  while (running_) {
    ssize_t bytes_read = read(client_fd, chunk, sizeof(chunk) - 1);
    if (bytes_read <= 0) break;
    chunk[bytes_read] = '\0';
    buffer.append(chunk);

    size_t pos = 0;
    while ((pos = buffer.find('\n')) != std::string::npos) {
      std::string line = buffer.substr(0, pos);
      buffer.erase(0, pos + 1);

      if (line.empty()) continue;

      try {
        auto req = json::parse(line);
        json resp = dispatchCommand(req, client_fd);
        if (!resp.empty()) {
          std::string resp_str = resp.dump() + "\n";
          (void)write(client_fd, resp_str.c_str(), resp_str.size());
        }
      } catch (const std::exception &e) {
        json err_resp = {{"status", "error"}, {"message", e.what()}};
        std::string err_str = err_resp.dump() + "\n";
        (void)write(client_fd, err_str.c_str(), err_str.size());
      }
    }
  }

  // Se o cliente desconectar, remove dos subscribers se estivesse cadastrado
  {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.erase(std::remove(subscribers_.begin(), subscribers_.end(), client_fd), subscribers_.end());
  }
  close(client_fd);
}

json MujocoIpcBridge::dispatchCommand(const json &req, int client_fd) {
  std::string cmd = req.value("cmd", "");

  if (cmd == "subscribe_state") {
    std::lock_guard<std::mutex> lock(subscribers_mutex_);
    subscribers_.push_back(client_fd);
    return {{"status", "ok"}, {"subscribed", true}, {"nq", m_->nq}, {"nv", m_->nv}};
  }

  if (cmd == "get_status") {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    return {
        {"status", "ok"},
        {"mode", mode_ == SimMode::FREE_RUN ? "free_run" : "step_mode"},
        {"paused", paused_.load()},
        {"sim_time", d_->time},
        {"timestep", m_->opt.timestep},
        {"nq", m_->nq},
        {"nv", m_->nv},
        {"nbody", m_->nbody},
        {"ngeom", m_->ngeom}};
  }

  if (cmd == "set_mode") {
    std::string mode_str = req.value("mode", "free_run");
    if (mode_str == "step_mode") {
      mode_ = SimMode::STEP_MODE;
    } else {
      mode_ = SimMode::FREE_RUN;
    }
    return {{"status", "ok"}, {"mode", mode_str}};
  }

  if (cmd == "set_speed") {
    double speed = req.value("speed", 1.0);
    setSpeedMultiplier(speed);
    return {{"status", "ok"}, {"speed_multiplier", speed_multiplier_.load()}};
  }

  if (cmd == "pause") {
    paused_ = true;
    return {{"status", "ok"}, {"paused", true}, {"sim_time", d_->time}};
  }

  if (cmd == "resume") {
    paused_ = false;
    mode_ = SimMode::FREE_RUN;
    return {{"status", "ok"}, {"paused", false}, {"mode", "free_run"}, {"sim_time", d_->time}};
  }

  if (cmd == "step") {
    int steps = req.value("steps", 1);
    if (mode_ != SimMode::STEP_MODE) {
      mode_ = SimMode::STEP_MODE;
    }
    {
      std::lock_guard<std::mutex> lock(step_mutex_);
      pending_steps_ += steps;
    }
    int wait_count = 0;
    while (pending_steps_ > 0 && wait_count++ < 200) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::lock_guard<std::mutex> lock(sim_mutex_);
    return {{"status", "ok"}, {"sim_time", d_->time}};
  }

  if (cmd == "reset") {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    mj_resetData(m_, d_);
    if (reset_callback_) {
      reset_callback_();
    }
    return {{"status", "ok"}, {"sim_time", d_->time}};
  }

  if (cmd == "get_bodies") {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    json bodies = json::array();
    for (int i = 0; i < m_->nbody; ++i) {
      const char *name = mj_id2name(m_, mjOBJ_BODY, i);
      std::string body_name = name ? name : "body_" + std::to_string(i);
      bodies.push_back({
          {"id", i},
          {"name", body_name},
          {"pos", {d_->xpos[3 * i], d_->xpos[3 * i + 1], d_->xpos[3 * i + 2]}},
          {"quat", {d_->xquat[4 * i], d_->xquat[4 * i + 1], d_->xquat[4 * i + 2], d_->xquat[4 * i + 3]}}
      });
    }
    return {{"status", "ok"}, {"bodies", bodies}};
  }

  if (cmd == "get_joints") {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    json joints = json::array();
    for (int i = 0; i < m_->njnt; ++i) {
      const char *name = mj_id2name(m_, mjOBJ_JOINT, i);
      std::string jnt_name = name ? name : "joint_" + std::to_string(i);
      int qpos_addr = m_->jnt_qposadr[i];
      int dof_addr = m_->jnt_dofadr[i];
      joints.push_back({
          {"id", i},
          {"name", jnt_name},
          {"qpos", d_->qpos[qpos_addr]},
          {"qvel", d_->qvel[dof_addr]}
      });
    }
    return {{"status", "ok"}, {"joints", joints}};
  }

  if (cmd == "apply_force") {
    std::string body_name = req.value("body", "");
    std::vector<double> force = req.value("force", std::vector<double>{0.0, 0.0, 0.0});
    std::vector<double> torque = req.value("torque", std::vector<double>{0.0, 0.0, 0.0});
    bool has_point = req.contains("point") && req["point"].is_array() && req["point"].size() == 3;

    std::lock_guard<std::mutex> lock(sim_mutex_);
    int body_id = mj_name2id(m_, mjOBJ_BODY, body_name.c_str());
    if (body_id < 0) {
      return {{"status", "error"}, {"message", "Body not found: " + body_name}};
    }

    if (has_point) {
      // Aplica a força no ponto exato 3D selecionado (como uma corda presa naquele ponto)
      mjtNum f[3] = {force[0], force[1], force[2]};
      mjtNum t[3] = {torque[0], torque[1], torque[2]};
      mjtNum p[3] = {req["point"][0].get<double>(), req["point"][1].get<double>(), req["point"][2].get<double>()};
      mj_applyFT(m_, d_, f, t, p, body_id, d_->qfrc_applied);
    } else {
      d_->xfrc_applied[6 * body_id + 0] = force[0];
      d_->xfrc_applied[6 * body_id + 1] = force[1];
      d_->xfrc_applied[6 * body_id + 2] = force[2];
      d_->xfrc_applied[6 * body_id + 3] = torque[0];
      d_->xfrc_applied[6 * body_id + 4] = torque[1];
      d_->xfrc_applied[6 * body_id + 5] = torque[2];
    }

    return {{"status", "ok"}, {"applied_to", body_name}};
  }

  return {{"status", "error"}, {"message", "Unknown command: " + cmd}};
}

} // namespace perse_mujoco
