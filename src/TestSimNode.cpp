#include "perse_mujoco/TestSimNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>

namespace perse_mujoco {

TestSimNode::TestSimNode() : Node("perse_mujoco_node") {
  std::string model_path =
      this->declare_parameter("model_path", rclcpp::PARAMETER_STRING)
          .get<std::string>();
  if (model_path.empty()) {
    std::string pkg_share =
        ament_index_cpp::get_package_share_directory("perse_mujoco");
    model_path = pkg_share + "/model/test_scene.xml";
  }

  std::string socket_path =
      this->declare_parameter("socket_path", rclcpp::PARAMETER_STRING)
          .get<std::string>();

  RCLCPP_INFO(this->get_logger(), "Carregando modelo MuJoCo: %s",
              model_path.c_str());

  char error[1000] = "Não foi possível carregar o arquivo XML";
  m_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  if (!m_) {
    RCLCPP_FATAL(this->get_logger(), "Erro ao carregar MuJoCo XML: %s", error);
    throw std::runtime_error("Falha ao inicializar modelo MuJoCo");
  }

  d_ = mj_makeData(m_);
  if (!d_) {
    RCLCPP_FATAL(this->get_logger(), "Erro ao alocar mjData");
    mj_deleteModel(m_);
    throw std::runtime_error("Falha ao inicializar dados do MuJoCo");
  }

  RCLCPP_INFO(
      this->get_logger(),
      "MuJoCo inicializado com sucesso! nq=%ld, nv=%ld, timestep=%.4f s",
      static_cast<long>(m_->nq), static_cast<long>(m_->nv), m_->opt.timestep);

  ipc_bridge_ =
      std::make_unique<MujocoIpcBridge>(m_, d_, sim_mutex_, socket_path);
  if (ipc_bridge_->start()) {
    RCLCPP_INFO(this->get_logger(), "IPC Bridge ativa em: %s",
                socket_path.c_str());
  }

  publish_clock_ = this->declare_parameter("publish_clock", true);
  if (publish_clock_) {
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::ClockQoS());
  }

  double rate_hz =
      this->declare_parameter("publish_rate_hz", rclcpp::PARAMETER_DOUBLE)
          .get<double>();
  auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);

  timer_ = this->create_wall_timer(
      timer_period, std::bind(&TestSimNode::stepSimulation, this));

  last_wall_time_ = this->now();
}

TestSimNode::~TestSimNode() {
  if (ipc_bridge_) {
    ipc_bridge_->stop();
  }
  std::lock_guard<std::mutex> lock(sim_mutex_);
  if (d_) {
    mj_deleteData(d_);
  }
  if (m_) {
    mj_deleteModel(m_);
  }
  RCLCPP_INFO(this->get_logger(), "Instância do MuJoCo finalizada.");
}

void TestSimNode::stepSimulation() {
  auto current_wall_time = this->now();
  double dt_wall = (current_wall_time - last_wall_time_).seconds();
  last_wall_time_ = current_wall_time;

  int steps_to_run = 0;
  if (ipc_bridge_->shouldStep(steps_to_run)) {
    if (ipc_bridge_->getMode() == MujocoIpcBridge::SimMode::FREE_RUN) {
      steps_to_run = std::max(1, static_cast<int>(dt_wall / m_->opt.timestep));
      steps_to_run = std::min(steps_to_run, 100);
    }

    std::lock_guard<std::mutex> lock(sim_mutex_);
    for (int i = 0; i < steps_to_run; ++i) {
      mj_step(m_, d_);
    }
    // Zera forças residuais após cada ciclo de passos para não acumular
    // aceleração infinita
    mju_zero(d_->qfrc_applied, m_->nv);
    mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  }

  // Transmite estado atualizado para visualizadores conectados (nativo e web)
  ipc_bridge_->broadcastState();

  if (publish_clock_ && clock_pub_) {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = rclcpp::Time(static_cast<int64_t>(d_->time * 1e9));
    clock_pub_->publish(clock_msg);
  }

  static int log_counter = 0;
  if (++log_counter % 50 == 0) {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    double box_z = d_->qpos[2];
    RCLCPP_INFO(this->get_logger(),
                "[SimTime: %.3f s] Mode: %s | test_box Z: %.3f m", d_->time,
                ipc_bridge_->getMode() == MujocoIpcBridge::SimMode::FREE_RUN
                    ? "FreeRun"
                    : "StepMode",
                box_z);
  }
}

} // namespace perse_mujoco
