#include "perse_mujoco/SyntheticVision.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>

namespace perse_mujoco {

SyntheticVision::SyntheticVision(rclcpp::Node *node, const mjModel *m,
                                 const std::string &mission_points_path)
    : node_(node) {
  std::string recognitions_topic =
      node_->declare_parameter("topics.bounding_boxes", rclcpp::PARAMETER_STRING).get<std::string>();

  detections_pub_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      recognitions_topic, rclcpp::SensorDataQoS().reliable());

  camera_site_id_ = mj_name2id(m, mjOBJ_SITE, "camera_site");
  chassis_body_id_ = mj_name2id(m, mjOBJ_BODY, "chassis");

  if (camera_site_id_ < 0) {
    RCLCPP_WARN(node_->get_logger(),
                "camera_site não encontrado no modelo MJCF! Detecção de visão desabilitada.");
  }

  // Calcula focal length a partir de HFOV e largura da imagem (416x416)
  fx_ = (image_width_ / 2.0) / std::tan(hfov_rad_ / 2.0);
  fy_ = fx_;
  cx_ = image_width_ / 2.0;
  cy_ = image_height_ / 2.0;

  // Carrega cones primeiro dos pontos de missão se fornecido
  std::string yaml_to_load = mission_points_path;
  if (yaml_to_load.empty()) {
    try {
      std::string bringup_share = ament_index_cpp::get_package_share_directory("perse_bringup");
      yaml_to_load = bringup_share + "/config/mission_points.yaml";
    } catch (...) {
      yaml_to_load = "";
    }
  }

  if (!yaml_to_load.empty()) {
    loadMissionPoints(yaml_to_load);
  }

  // Também busca corpos de cones presentes diretamente no modelo MuJoCo
  findConesInModel(m);

  RCLCPP_INFO(node_->get_logger(),
              "SyntheticVision inicializada com %zu alvos de cone! FOV: %.1f deg, Resolução: %.0fx%.0f",
              cones_.size(), hfov_rad_ * 180.0 / M_PI, image_width_, image_height_);
}

void SyntheticVision::loadMissionPoints(const std::string &yaml_path) {
  try {
    std::ifstream f(yaml_path);
    if (!f.good()) {
      return;
    }
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["points"] && config["points"].IsSequence()) {
      int idx = 1;
      for (const auto &pt : config["points"]) {
        bool has_cone = pt["has_cone"] ? pt["has_cone"].as<bool>() : false;
        if (has_cone) {
          ConeTarget cone;
          cone.name = "mission_cone_" + std::to_string(idx);
          cone.x = pt["x"].as<double>();
          cone.y = pt["y"].as<double>();
          cone.z = 0.15;
          cone.body_id = -1;
          cones_.push_back(cone);
        }
        idx++;
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Não foi possível carregar mission_points no SyntheticVision: %s",
                e.what());
  }
}

void SyntheticVision::findConesInModel(const mjModel *m) {
  if (!m) return;
  for (int i = 0; i < m->nbody; ++i) {
    const char *name = mj_id2name(m, mjOBJ_BODY, i);
    if (name && std::string(name).rfind("cone", 0) == 0) {
      // Verifica se já não foi cadastrado
      bool exists = false;
      for (const auto &c : cones_) {
        if (c.body_id == i) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        ConeTarget cone;
        cone.name = name;
        cone.body_id = i;
        cone.x = 0.0;
        cone.y = 0.0;
        cone.z = 0.15;
        cones_.push_back(cone);
      }
    }
  }
}

const std::vector<ConeTarget> &SyntheticVision::getCones() const {
  return cones_;
}

void SyntheticVision::updateAndPublish(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  if (!detections_pub_ || camera_site_id_ < 0) return;

  if (stamp < last_publish_time_) {
    last_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  double dt = (stamp - last_publish_time_).seconds();
  if (dt < 1.0 / publish_rate_hz_) {
    return;
  }
  last_publish_time_ = stamp;

  vision_msgs::msg::Detection2DArray detections_msg;
  detections_msg.header.stamp = stamp;
  detections_msg.header.frame_id = "camera_link";

  computeDetections(m, d, stamp, detections_msg);
  detections_pub_->publish(detections_msg);
}

void SyntheticVision::computeDetections(const mjModel *m, const mjData *d, const rclcpp::Time &stamp,
                                       vision_msgs::msg::Detection2DArray &out_detections) {
  (void)stamp;
  out_detections.detections.clear();
  if (camera_site_id_ < 0) return;

  // Posição e orientação da câmera no mundo
  const mjtNum *cam_pos = &d->site_xpos[camera_site_id_ * 3];
  const mjtNum *cam_mat = &d->site_xmat[camera_site_id_ * 9];

  for (const auto &cone : cones_) {
    double cone_x = cone.x;
    double cone_y = cone.y;
    double cone_z = cone.z;

    if (cone.body_id >= 0) {
      cone_x = d->xpos[cone.body_id * 3 + 0];
      cone_y = d->xpos[cone.body_id * 3 + 1];
      cone_z = d->xpos[cone.body_id * 3 + 2];
    }

    // Vetor câmera -> cone no mundo
    mjtNum dx = cone_x - cam_pos[0];
    mjtNum dy = cone_y - cam_pos[1];
    mjtNum dz = cone_z - cam_pos[2];

    // Transforma para coordenadas locais da câmera
    // cam_mat armazena colunas X, Y, Z da câmera
    double rx = cam_mat[0] * dx + cam_mat[3] * dy + cam_mat[6] * dz; // Linha de visão frontal
    double ry = cam_mat[1] * dx + cam_mat[4] * dy + cam_mat[7] * dz; // Eixo lateral esquerdo
    double rz = cam_mat[2] * dx + cam_mat[5] * dy + cam_mat[8] * dz; // Eixo vertical para cima

    // Rejeita cones atrás da câmera ou fora do alcance
    if (rx < min_detect_dist_ || rx > max_detect_dist_) {
      continue;
    }

    // Projeta em coordenadas de pixels (416x416)
    double u = cx_ - fx_ * (ry / rx);
    double v = cy_ - fy_ * (rz / rx);

    double bbox_w = fx_ * (cone_width_ / rx);
    double bbox_h = fy_ * (cone_height_ / rx);

    // Verifica se a bounding box cruza a imagem
    if (u + bbox_w / 2.0 < 0 || u - bbox_w / 2.0 > image_width_ ||
        v + bbox_h / 2.0 < 0 || v - bbox_h / 2.0 > image_height_) {
      continue;
    }

    // Teste de oclusão via mj_ray()
    mjtNum dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1e-4) continue;

    mjtNum ray_dir[3] = {dx / dist, dy / dist, dz / dist};
    int hit_geom = -1;
    mjtNum hit_dist = mj_ray(m, d, cam_pos, ray_dir, nullptr, 1, chassis_body_id_, &hit_geom, nullptr);

    if (hit_dist >= 0 && hit_dist < dist - 0.25) {
      // O raio colidiu antes do cone. Verifica se a geometria atingida pertence ao cone
      if (hit_geom >= 0 && cone.body_id >= 0) {
        int hit_body = m->geom_bodyid[hit_geom];
        if (hit_body != cone.body_id) {
          continue; // Cone ocluso por outro objeto!
        }
      } else {
        continue; // Ocluso!
      }
    }

    // Cone visível e não ocluso! Constrói a mensagem Detection2D
    vision_msgs::msg::Detection2D det;
    det.header.stamp = out_detections.header.stamp;
    det.header.frame_id = "camera_link";

    det.bbox.center.position.x = u;
    det.bbox.center.position.y = v;
    det.bbox.size_x = bbox_w;
    det.bbox.size_y = bbox_h;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = "traffic_cone";
    hyp.hypothesis.score = 0.95;
    det.results.push_back(hyp);

    out_detections.detections.push_back(det);
  }
}

} // namespace perse_mujoco
