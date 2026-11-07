#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <mujoco/mujoco.h>

#include <string>
#include <vector>

namespace perse_mujoco {

struct ConeTarget {
  std::string name;
  double x{0.0};
  double y{0.0};
  double z{0.15};
  int body_id{-1};
};

class SyntheticVision {
public:
  SyntheticVision(rclcpp::Node *node, const mjModel *m, const std::string &mission_points_path = "");
  ~SyntheticVision() = default;

  // Atualiza as detecções geométricas sintéticas e publica se no período apropriado
  void updateAndPublish(const mjModel *m, const mjData *d, const rclcpp::Time &stamp);

  // Computa as detecções instantâneas
  void computeDetections(const mjModel *m, const mjData *d, const rclcpp::Time &stamp,
                         vision_msgs::msg::Detection2DArray &out_detections);

  const std::vector<ConeTarget> &getCones() const;

private:
  void loadMissionPoints(const std::string &yaml_path);
  void findConesInModel(const mjModel *m);

  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;

  std::vector<ConeTarget> cones_;

  int camera_site_id_{-1};
  int chassis_body_id_{-1};

  // Parâmetros intrínsecos da câmera simulada
  double image_width_{416.0};
  double image_height_{416.0};
  double hfov_rad_{1.2217}; // ~70 graus
  double fx_{297.05};
  double fy_{297.05};
  double cx_{208.0};
  double cy_{208.0};

  double min_detect_dist_{0.2};
  double max_detect_dist_{10.0};
  double cone_height_{0.3};
  double cone_width_{0.2};

  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  double publish_rate_hz_{25.0};
};

} // namespace perse_mujoco
