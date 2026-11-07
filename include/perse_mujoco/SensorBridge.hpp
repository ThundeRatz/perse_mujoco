#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <mujoco/mujoco.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "perse_mujoco/SyntheticVision.hpp"

#include <memory>
#include <string>
#include <vector>

namespace perse_mujoco {

class SensorBridge {
public:
  SensorBridge(rclcpp::Node *node, const mjModel *m,
               std::shared_ptr<SyntheticVision> synthetic_vision,
               double wheel_radius);
  ~SensorBridge() = default;

  // Atualiza e publica todos os sensores de acordo com suas taxas de amostragem
  void updateAndPublish(const mjModel *m, const mjData *d,
                        const rclcpp::Time &stamp);

  // Força publicação imediata do /clock
  void publishClock(const mjData *d);

private:
  void publishImu(const mjModel *m, const mjData *d, const rclcpp::Time &stamp);
  void publishEncoder(const mjModel *m, const mjData *d,
                      const rclcpp::Time &stamp);
  void publishBumper(const mjModel *m, const mjData *d,
                     const rclcpp::Time &stamp);
  void publishInductive(const mjModel *m, const mjData *d,
                        const rclcpp::Time &stamp);
  void publishLidar(const mjModel *m, const mjData *d,
                    const rclcpp::Time &stamp);
  void publishOdomGt(const mjModel *m, const mjData *d,
                     const rclcpp::Time &stamp);

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<SyntheticVision> synthetic_vision_;

  // Publicadores ROS 2
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inductive_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gt_pub_;

  // Identificadores de sensores e corpos no MuJoCo
  int imu_accel_id_{-1};
  int imu_gyro_id_{-1};
  int imu_site_id_{-1};
  int chassis_body_id_{-1};

  int wheel_rl_jnt_id_{-1};
  int wheel_rl_qpos_adr_{-1};

  int bumper_jnt_id_{-1};
  int bumper_qpos_adr_{-1};
  int bumper_body_id_{-1};

  int inductive_site_id_{-1};
  int lidar_site_id_{-1};

  // Parâmetros e estados internos
  double wheel_radius_{0.0714};
  double last_encoder_dist_{0.0};
  bool last_bumper_state_{false};
  bool last_inductive_state_{false};

  // Temporizadores para taxas de publicação
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_encoder_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_bumper_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_inductive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_gt_time_{0, 0, RCL_ROS_TIME};

  double imu_rate_hz_{100.0};
  double encoder_rate_hz_{100.0};
  double bumper_rate_hz_{100.0};
  double inductive_rate_hz_{50.0};
  double lidar_rate_hz_{10.0};
  double odom_gt_rate_hz_{50.0};

  // Constantes de detecção
  double inductive_plate_size_{1.0};        // Placa 1x1m
  double encoder_resolution_{0.001};        // Resolução de quantização de 1mm
  double bumper_trigger_threshold_{-0.005}; // Contração em metros
};

} // namespace perse_mujoco
