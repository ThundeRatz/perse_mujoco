#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

namespace perse_mujoco {

class TestSimNode : public rclcpp::Node {
public:
  TestSimNode() : Node("perse_mujoco_node") {
    // Parâmetros de simulação
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<bool>("publish_clock", true);

    std::string model_path = this->get_parameter("model_path").as_string();
    if (model_path.empty()) {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("perse_mujoco");
      model_path = pkg_share + "/model/test_scene.xml";
    }

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
                "MuJoCo inicializado com sucesso! nq=%ld, nv=%ld, timestep=%.4f s",
                static_cast<long>(m_->nq), static_cast<long>(m_->nv), m_->opt.timestep);

    // Publicador de relógio de simulação (/clock)
    publish_clock_ = this->get_parameter("publish_clock").as_bool();
    if (publish_clock_) {
      clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
          "/clock", rclcpp::ClockQoS());
    }

    // Timer do loop de simulação e controle ROS
    double rate_hz = this->get_parameter("publish_rate_hz").as_double();
    auto timer_period = std::chrono::duration<double>(1.0 / rate_hz);

    timer_ = this->create_wall_timer(
        timer_period, std::bind(&TestSimNode::stepSimulation, this));

    last_wall_time_ = this->now();
  }

  ~TestSimNode() override {
    if (d_) {
      mj_deleteData(d_);
    }
    if (m_) {
      mj_deleteModel(m_);
    }
    RCLCPP_INFO(this->get_logger(), "Instância do MuJoCo finalizada.");
  }

private:
  void stepSimulation() {
    auto current_wall_time = this->now();
    double dt_wall = (current_wall_time - last_wall_time_).seconds();
    last_wall_time_ = current_wall_time;

    // Avança a física até igualar o tempo de parede (ou proporcional ao timestep)
    // Para estabilidade, avançamos steps fixos equivalentes ao ciclo
    int steps_to_run = std::max(1, static_cast<int>(dt_wall / m_->opt.timestep));
    // Limite para evitar espiral de atraso (catch-up)
    steps_to_run = std::min(steps_to_run, 100);

    for (int i = 0; i < steps_to_run; ++i) {
      mj_step(m_, d_);
    }

    // Publica /clock
    if (publish_clock_ && clock_pub_) {
      rosgraph_msgs::msg::Clock clock_msg;
      clock_msg.clock = rclcpp::Time(static_cast<int64_t>(d_->time * 1e9));
      clock_pub_->publish(clock_msg);
    }

    // Log periódico de telemetria básica (posição da caixa caindo)
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {
      // test_box é o primeiro corpo após world (id 1)
      double box_z = d_->qpos[2];
      RCLCPP_INFO(this->get_logger(),
                  "[SimTime: %.3f s] Passos executados: %d | test_box Z: %.3f m",
                  d_->time, steps_to_run, box_z);
    }
  }

  mjModel *m_{nullptr};
  mjData *d_{nullptr};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_wall_time_;
  bool publish_clock_{true};
};

} // namespace perse_mujoco

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<perse_mujoco::TestSimNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("perse_mujoco"), "Exceção fatal: %s",
                 e.what());
  }
  rclcpp::shutdown();
  return 0;
}
