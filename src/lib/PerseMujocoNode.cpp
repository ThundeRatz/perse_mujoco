#include "perse_mujoco/PerseMujocoNode.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <stdexcept>
#include <std_srvs/srv/trigger.hpp>

namespace perse_mujoco {

std::string PerseMujocoNode::resolveModelPath(const std::string &default_model_path,
                                              const std::string &generate_service_name) {
  // Cria cliente para o serviço de geração de mapas
  auto client = this->create_client<std_srvs::srv::Trigger>(generate_service_name);

  RCLCPP_INFO(this->get_logger(),
              "Aguardando gerador de mapas no serviço '%s' (timeout: 1.5s)...",
              generate_service_name.c_str());

  if (client->wait_for_service(std::chrono::milliseconds(1500))) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);

    // Aguarda a resposta do serviço
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::milliseconds(2000)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = future.get();
      if (resp->success) {
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("perse_mujoco");
        std::string generated_path = pkg_share + "/model/generated_scene.xml";

        // Verifica se o arquivo gerado realmente existe e é legível
        std::ifstream check_file(generated_path);
        if (check_file.good()) {
          RCLCPP_INFO(this->get_logger(),
                      "Mapa gerado recebido com sucesso: %s",
                      generated_path.c_str());
          return generated_path;
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Serviço respondeu sucesso, mas arquivo gerado não foi encontrado em '%s'. Usando cena padrão.",
                      generated_path.c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Gerador de mapas retornou falha (%s). Usando cena padrão.",
                    resp->message.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Timeout ao aguardar resposta do gerador de mapas. Usando cena padrão.");
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Gerador de mapas ausente ou inativo. Carregando cena padrão.");
  }

  return default_model_path;
}

PerseMujocoNode::PerseMujocoNode() : Node("perse_mujoco_node") {
  std::string pkg_share =
      ament_index_cpp::get_package_share_directory("perse_mujoco");
  std::string default_scene_path = pkg_share + "/model/default_scene.xml";

  std::string model_path_param =
      this->declare_parameter("model_path", rclcpp::PARAMETER_STRING)
          .get<std::string>();

  std::string generate_srv_name =
      this->declare_parameter("services.generate_map", rclcpp::PARAMETER_STRING)
          .get<std::string>();

  std::string model_path = model_path_param;
  if (model_path.empty()) {
    // Tenta obter o mapa gerado pelo map_generator_node via ROS 2 Service Trigger
    model_path = resolveModelPath(default_scene_path, generate_srv_name);
  }

  std::string mission_points =
      this->declare_parameter("mission_points", rclcpp::PARAMETER_STRING)
          .get<std::string>();
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

  RCLCPP_INFO(this->get_logger(),
              "MuJoCo carregado com sucesso! nq=%ld, nv=%ld, timestep=%.4f s",
              static_cast<long>(m_->nq), static_cast<long>(m_->nv),
              m_->opt.timestep);

  // Inicializa o IPC Bridge para o Visualizador e MCP
  ipc_bridge_ =
      std::make_unique<MujocoIpcBridge>(m_, d_, sim_mutex_, socket_path);
  if (ipc_bridge_->start()) {
    RCLCPP_INFO(this->get_logger(), "IPC Bridge ativa em: %s",
                socket_path.c_str());
  }

  // Inicializa subsistemas ROS 2
  actuator_bridge_ = std::make_unique<ActuatorBridge>(this, m_);
  synthetic_vision_ =
      std::make_shared<SyntheticVision>(this, m_, mission_points);
  sensor_bridge_ = std::make_unique<SensorBridge>(
      this, m_, synthetic_vision_, actuator_bridge_->getWheelRadius());

  // Conecta reset do IPC ao ActuatorBridge para garantir que alvos de controle
  // sejam zerados
  ipc_bridge_->setResetCallback([this]() {
    if (actuator_bridge_) {
      actuator_bridge_->reset();
    }
  });

  publish_clock_ = this->declare_parameter("publish_clock", true);

  double rate_hz =
      this->declare_parameter("publish_rate_hz", rclcpp::PARAMETER_DOUBLE)
          .get<double>();
  auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);

  timer_ = this->create_wall_timer(
      timer_period, std::bind(&PerseMujocoNode::stepSimulation, this));

  last_wall_time_ = std::chrono::steady_clock::now();
  last_fps_time_ = last_wall_time_;
  steps_since_fps_ = 0;
  RCLCPP_INFO(this->get_logger(),
              "PerseMujocoNode inicializado com sucesso a %.1f Hz!", rate_hz);
}

PerseMujocoNode::~PerseMujocoNode() {
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
  RCLCPP_INFO(this->get_logger(), "Instância PerseMujocoNode finalizada.");
}

void PerseMujocoNode::stepSimulation() {
  auto current_wall_time = std::chrono::steady_clock::now();
  double dt_wall =
      std::chrono::duration<double>(current_wall_time - last_wall_time_)
          .count();
  last_wall_time_ = current_wall_time;

  int steps_to_run = 0;
  if (ipc_bridge_->shouldStep(steps_to_run)) {
    if (ipc_bridge_->getMode() == MujocoIpcBridge::SimMode::FREE_RUN) {
      double mult = ipc_bridge_->getSpeedMultiplier();
      // Acumula tempo físico real decorrido (steady_clock) ponderado pelo
      // multiplicador de velocidade
      time_accumulator_ += dt_wall * mult;

      // Proteção contra espiral de passos caso haja atraso extremo no loop
      if (time_accumulator_ > 0.5) {
        time_accumulator_ = 0.5;
      }

      steps_to_run = 0;
      while (time_accumulator_ >= m_->opt.timestep) {
        time_accumulator_ -= m_->opt.timestep;
        steps_to_run++;
        if (steps_to_run >= 200) {
          time_accumulator_ = 0.0;
          break;
        }
      }
    }

    if (steps_to_run > 0) {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      for (int i = 0; i < steps_to_run; ++i) {
        // Aplica atuações mais recentes antes do passo de física
        actuator_bridge_->applyControls(m_, d_);

        mj_step(m_, d_);
      }

      steps_since_fps_ += steps_to_run;

      // Zera forças residuais após cada ciclo
      mju_zero(d_->qfrc_applied, m_->nv);
      mju_zero(d_->xfrc_applied, 6 * m_->nbody);
    }
  } else {
    // Se estiver pausado, zera o acumulador para não dar um salto temporal ao
    // retomar
    time_accumulator_ = 0.0;
  }

  // Atualiza medição real de taxa de atualização da física (Physics Hz em
  // relação ao steady_clock)
  double elapsed_fps =
      std::chrono::duration<double>(current_wall_time - last_fps_time_).count();
  if (elapsed_fps >= 0.25) {
    double current_physics_fps =
        static_cast<double>(steps_since_fps_) / elapsed_fps;
    ipc_bridge_->updatePhysicsFps(current_physics_fps);
    steps_since_fps_ = 0;
    last_fps_time_ = current_wall_time;
  }

  // Transmite estado atualizado para visualizadores conectados (nativo e web)
  ipc_bridge_->broadcastState();

  // Atualiza e publica todos os sensores ROS 2
  rclcpp::Time current_stamp(static_cast<int64_t>(d_->time * 1e9),
                             RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sensor_bridge_->updateAndPublish(m_, d_, current_stamp);
    synthetic_vision_->updateAndPublish(m_, d_, current_stamp);
  }
}

} // namespace perse_mujoco
