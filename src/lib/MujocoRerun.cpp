#include "perse_mujoco/MujocoRerun.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <rerun/archetypes/clear.hpp>
#include <rerun/archetypes/coordinate_frame.hpp>
#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/pinhole.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <vector>

namespace perse_mujoco {

using json = nlohmann::json;

MujocoRerun::MujocoRerun() : Node("perse_mujoco_rerun") {
  // Conforme AGENTS.md: declaração inline obrigatória sem defaults numéricos no
  // C++
  model_path_ = this->declare_parameter("model_path", rclcpp::PARAMETER_STRING)
                    .get<std::string>();
  sim_socket_path_ =
      this->declare_parameter("socket_path", rclcpp::PARAMETER_STRING)
          .get<std::string>();
  entity_root_ =
      this->declare_parameter("rerun.entity_root", rclcpp::PARAMETER_STRING)
          .get<std::string>();
  publish_rate_hz_ =
      this->declare_parameter("rerun.publish_rate_hz", rclcpp::PARAMETER_DOUBLE)
          .get<double>();
  recognitions_topic_ =
      this->declare_parameter("topics.bounding_boxes", rclcpp::PARAMETER_STRING)
          .get<std::string>();

  recognitions_sub_ =
      this->create_subscription<vision_msgs::msg::Detection2DArray>(
          recognitions_topic_, rclcpp::SensorDataQoS().reliable(),
          std::bind(&MujocoRerun::recognitionsCallback, this,
                    std::placeholders::_1));

  if (model_path_.empty()) {
    try {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("perse_mujoco");
      std::string gen_path = pkg_share + "/model/generated_scene.xml";
      FILE *f = fopen(gen_path.c_str(), "r");
      if (f) {
        fclose(f);
        model_path_ = gen_path;
      } else {
        model_path_ = pkg_share + "/model/default_scene.xml";
      }
    } catch (...) {
      model_path_ = "model/default_scene.xml";
    }
  }
}

MujocoRerun::~MujocoRerun() {
  running_ = false;
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }
  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}

bool MujocoRerun::init() {
  // Inicializa o cliente Rerun usando a estratégia compartilhada em perse_lib
  rerun_client_ =
      std::make_shared<RerunClient>(shared_from_this(), "perse_mujoco_rerun");

  char error[1000] = "Não foi possível carregar o arquivo XML";
  m_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
  if (!m_) {
    RCLCPP_ERROR(this->get_logger(), "Falha ao carregar modelo MuJoCo '%s': %s",
                 model_path_.c_str(), error);
    return false;
  }

  d_ = mj_makeData(m_);
  if (!d_) {
    RCLCPP_ERROR(this->get_logger(), "Falha ao alocar mjData.");
    return false;
  }

  mj_forward(m_, d_);

  // Aguarda o Rerun estar pronto para enviar a geometria estática
  if (rerun_client_ && rerun_client_->ok()) {
    logModel();
  } else {
    RCLCPP_WARN(this->get_logger(),
                "RerunStreamProxy não conectado inicialmente. O modelo será "
                "enviado assim que conectar.");
  }

  running_ = true;
  sync_thread_ = std::thread(&MujocoRerun::ipcSyncLoop, this);
  return true;
}

void MujocoRerun::logGeomWithFrame(const std::string &entity_path, int geom_id,
                                   const std::string &parent_frame) {
  if (!rerun_client_ || !rerun_client_->ok()) {
    return;
  }

  auto stream = rerun_client_->stream();

  // Define CoordinateFrame associado ao corpo pai (tf#/mujoco/bodies/...)
  stream->log_static(entity_path, rerun::CoordinateFrame(parent_frame));

  // Posição e orientação local da geometria no corpo pai
  mjtNum *g_pos = m_->geom_pos + 3 * geom_id;
  mjtNum *g_quat = m_->geom_quat + 4 * geom_id; // MuJoCo wxyz

  rerun::components::Translation3D trans(static_cast<float>(g_pos[0]),
                                         static_cast<float>(g_pos[1]),
                                         static_cast<float>(g_pos[2]));
  rerun::encodings::Quaternion quat = rerun::encodings::Quaternion::from_wxyz(
      static_cast<float>(g_quat[0]), static_cast<float>(g_quat[1]),
      static_cast<float>(g_quat[2]), static_cast<float>(g_quat[3]));

  stream->log_static(
      entity_path,
      rerun::InstancePoses3D().with_translations({trans}).with_quaternions(
          {quat}));

  // Material e cor do geom (seguindo referência rerun-loader-mjcf)
  int mat_id = m_->geom_matid[geom_id];
  int tex_id = -1;
  if (mat_id >= 0 && mat_id < m_->nmat) {
    // mjTEXROLE_RGB = 1 em mjtype.h
    tex_id = m_->mat_texid[mat_id * 10 + 1];
  }

  float r = (mat_id >= 0 && mat_id < m_->nmat) ? m_->mat_rgba[4 * mat_id + 0]
                                               : m_->geom_rgba[4 * geom_id + 0];
  float g = (mat_id >= 0 && mat_id < m_->nmat) ? m_->mat_rgba[4 * mat_id + 1]
                                               : m_->geom_rgba[4 * geom_id + 1];
  float b = (mat_id >= 0 && mat_id < m_->nmat) ? m_->mat_rgba[4 * mat_id + 2]
                                               : m_->geom_rgba[4 * geom_id + 2];
  float a = (mat_id >= 0 && mat_id < m_->nmat) ? m_->mat_rgba[4 * mat_id + 3]
                                               : m_->geom_rgba[4 * geom_id + 3];

  uint8_t ur = static_cast<uint8_t>(std::clamp(r * 255.0f, 0.0f, 255.0f));
  uint8_t ug = static_cast<uint8_t>(std::clamp(g * 255.0f, 0.0f, 255.0f));
  uint8_t ub = static_cast<uint8_t>(std::clamp(b * 255.0f, 0.0f, 255.0f));
  uint8_t ua = static_cast<uint8_t>(std::clamp(a * 255.0f, 0.0f, 255.0f));
  rerun::Rgba32 color(ur, ug, ub, ua);

  int geom_type = m_->geom_type[geom_id];
  mjtNum *size = m_->geom_size + 3 * geom_id;

  switch (geom_type) {
  case mjGEOM_PLANE: {
    float extent =
        static_cast<float>(m_->stat.extent > 1.0 ? m_->stat.extent : 1.0);
    float hx = static_cast<float>(size[0] > 0 ? size[0] : extent);
    float hy = static_cast<float>(size[1] > 0 ? size[1] : extent);

    std::vector<rerun::Position3D> vertices = {
        {-hx, -hy, 0.0f}, {hx, -hy, 0.0f}, {hx, hy, 0.0f}, {-hx, hy, 0.0f}};
    std::vector<rerun::TriangleIndices> faces = {{0, 1, 2}, {0, 2, 3}};
    std::vector<rerun::Vector3D> normals = {{0.0f, 0.0f, 1.0f},
                                            {0.0f, 0.0f, 1.0f},
                                            {0.0f, 0.0f, 1.0f},
                                            {0.0f, 0.0f, 1.0f}};

    if (tex_id >= 0 && tex_id < m_->ntex) {
      int tex_w = m_->tex_width[tex_id];
      int tex_h = m_->tex_height[tex_id];
      int tex_nc = m_->tex_nchannel[tex_id];
      size_t tex_adr = m_->tex_adr[tex_id];

      // Mapeamento UV conforme rerun-loader-mjcf
      float texrepeat_x =
          (mat_id >= 0) ? m_->mat_texrepeat[2 * mat_id + 0] : 1.0f;
      float texrepeat_y =
          (mat_id >= 0) ? m_->mat_texrepeat[2 * mat_id + 1] : 1.0f;
      bool texuniform = (mat_id >= 0) && (m_->mat_texuniform[mat_id] != 0);
      bool is_checker = (m_->tex_pathadr[tex_id] < 0);

      float scl_x = 1.0f;
      float scl_y = 1.0f;
      if (is_checker) {
        if (texuniform) {
          scl_x = (texrepeat_x * 2.0f * hx) / 2.0f;
          scl_y = (texrepeat_y * 2.0f * hy) / 2.0f;
        } else {
          scl_x = texrepeat_x / 2.0f;
          scl_y = texrepeat_y / 2.0f;
        }
      }

      std::vector<rerun::components::Texcoord2D> uvs;
      if (is_checker) {
        uvs = {
            {-0.5f * scl_x - 0.5f, 0.5f * scl_y - 0.5f},
            {0.5f * scl_x - 0.5f, 0.5f * scl_y - 0.5f},
            {0.5f * scl_x - 0.5f, -0.5f * scl_y - 0.5f},
            {-0.5f * scl_x - 0.5f, -0.5f * scl_y - 0.5f},
        };
      } else {
        float uv_max_x = texuniform ? (texrepeat_x * 2.0f * hx) : texrepeat_x;
        float uv_max_y = texuniform ? (texrepeat_y * 2.0f * hy) : texrepeat_y;
        uvs = {
            {0.0f, uv_max_y},
            {uv_max_x, uv_max_y},
            {uv_max_x, 0.0f},
            {0.0f, 0.0f},
        };
      }

      // Buffer de pixels da textura
      size_t byte_size = static_cast<size_t>(tex_w) * tex_h * tex_nc;
      std::vector<uint8_t> raw_pixels(m_->tex_data + tex_adr,
                                      m_->tex_data + tex_adr + byte_size);

      rerun::components::ImageBuffer img_buf(raw_pixels);
      rerun::encodings::ColorModel col_model =
          (tex_nc == 4)   ? rerun::encodings::ColorModel::RGBA
          : (tex_nc == 3) ? rerun::encodings::ColorModel::RGB
                          : rerun::encodings::ColorModel::L;

      rerun::components::ImageFormat img_fmt(
          rerun::WidthHeight(static_cast<uint32_t>(tex_w),
                             static_cast<uint32_t>(tex_h)),
          col_model, rerun::encodings::ChannelDatatype::U8);

      stream->log_static(entity_path, rerun::Mesh3D(vertices)
                                          .with_triangle_indices(faces)
                                          .with_vertex_normals(normals)
                                          .with_vertex_texcoords(uvs)
                                          .with_albedo_texture_buffer(img_buf)
                                          .with_albedo_texture_format(img_fmt));
    } else {
      std::vector<rerun::Color> colors = {color, color, color, color};
      stream->log_static(entity_path, rerun::Mesh3D(vertices)
                                          .with_triangle_indices(faces)
                                          .with_vertex_normals(normals)
                                          .with_vertex_colors(colors));
    }
    break;
  }
  case mjGEOM_SPHERE: {
    float radius = static_cast<float>(size[0]);
    stream->log_static(
        entity_path,
        rerun::Ellipsoids3D::from_half_sizes({{radius, radius, radius}})
            .with_colors({color})
            .with_fill_mode(rerun::components::FillMode::Solid));
    break;
  }
  case mjGEOM_ELLIPSOID: {
    float rx = static_cast<float>(size[0]);
    float ry = static_cast<float>(size[1]);
    float rz = static_cast<float>(size[2]);
    stream->log_static(entity_path,
                       rerun::Ellipsoids3D::from_half_sizes({{rx, ry, rz}})
                           .with_colors({color})
                           .with_fill_mode(rerun::components::FillMode::Solid));
    break;
  }
  case mjGEOM_BOX: {
    float hx = static_cast<float>(size[0]);
    float hy = static_cast<float>(size[1]);
    float hz = static_cast<float>(size[2]);

    // Box mesh com 6 faces (24 vértices) e normais para iluminação e sombras
    std::vector<rerun::Position3D> vertices = {// +X face
                                               {hx, -hy, -hz},
                                               {hx, hy, -hz},
                                               {hx, hy, hz},
                                               {hx, -hy, hz},
                                               // -X face
                                               {-hx, hy, -hz},
                                               {-hx, -hy, -hz},
                                               {-hx, -hy, hz},
                                               {-hx, hy, hz},
                                               // +Y face
                                               {hx, hy, -hz},
                                               {-hx, hy, -hz},
                                               {-hx, hy, hz},
                                               {hx, hy, hz},
                                               // -Y face
                                               {-hx, -hy, -hz},
                                               {hx, -hy, -hz},
                                               {hx, -hy, hz},
                                               {-hx, -hy, hz},
                                               // +Z face
                                               {-hx, -hy, hz},
                                               {hx, -hy, hz},
                                               {hx, hy, hz},
                                               {-hx, hy, hz},
                                               // -Z face
                                               {-hx, hy, -hz},
                                               {hx, hy, -hz},
                                               {hx, -hy, -hz},
                                               {-hx, -hy, -hz}};

    std::vector<rerun::Vector3D> normals = {
        {1.0f, 0.0f, 0.0f},  {1.0f, 0.0f, 0.0f},  {1.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f},
        {-1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},  {0.0f, 1.0f, 0.0f},  {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f},
        {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, -1.0f}};

    std::vector<rerun::TriangleIndices> faces;
    for (uint32_t i = 0; i < 6; ++i) {
      faces.push_back({i * 4 + 0, i * 4 + 1, i * 4 + 2});
      faces.push_back({i * 4 + 0, i * 4 + 2, i * 4 + 3});
    }

    std::vector<rerun::Color> colors(24, color);

    stream->log_static(entity_path, rerun::Mesh3D(vertices)
                                        .with_triangle_indices(faces)
                                        .with_vertex_normals(normals)
                                        .with_vertex_colors(colors));
    break;
  }
  case mjGEOM_CYLINDER: {
    float radius = static_cast<float>(size[0]);
    float half_h = static_cast<float>(size[1]);
    stream->log_static(
        entity_path,
        rerun::Cylinders3D::from_lengths_and_radii({2.0f * half_h}, {radius})
            .with_centers({{0.0f, 0.0f, 0.0f}})
            .with_colors({color})
            .with_fill_mode(rerun::components::FillMode::Solid));
    break;
  }
  case mjGEOM_CAPSULE: {
    float radius = static_cast<float>(size[0]);
    float half_h = static_cast<float>(size[1]);
    stream->log_static(
        entity_path,
        rerun::Capsules3D::from_lengths_and_radii({2.0f * half_h}, {radius})
            .with_translations({{0.0f, 0.0f, -half_h}})
            .with_colors({color})
            .with_fill_mode(rerun::components::FillMode::Solid));
    break;
  }
  case mjGEOM_MESH: {
    int mesh_id = m_->geom_dataid[geom_id];
    if (mesh_id < 0 || mesh_id >= m_->nmesh) {
      break;
    }
    int vert_adr = m_->mesh_vertadr[mesh_id];
    int face_adr = m_->mesh_faceadr[mesh_id];
    int face_num = m_->mesh_facenum[mesh_id];
    int norm_adr = m_->mesh_normaladr[mesh_id];

    // Cria mesh explodida para preservar normais por vértice/face
    std::vector<rerun::Position3D> vertices;
    std::vector<rerun::TriangleIndices> faces;
    std::vector<rerun::Vector3D> normals;
    vertices.reserve(face_num * 3);
    faces.reserve(face_num);
    normals.reserve(face_num * 3);

    for (int f = 0; f < face_num; ++f) {
      int idx0 = m_->mesh_face[3 * (face_adr + f) + 0];
      int idx1 = m_->mesh_face[3 * (face_adr + f) + 1];
      int idx2 = m_->mesh_face[3 * (face_adr + f) + 2];

      float v0[3] = {m_->mesh_vert[3 * (vert_adr + idx0) + 0],
                     m_->mesh_vert[3 * (vert_adr + idx0) + 1],
                     m_->mesh_vert[3 * (vert_adr + idx0) + 2]};
      float v1[3] = {m_->mesh_vert[3 * (vert_adr + idx1) + 0],
                     m_->mesh_vert[3 * (vert_adr + idx1) + 1],
                     m_->mesh_vert[3 * (vert_adr + idx1) + 2]};
      float v2[3] = {m_->mesh_vert[3 * (vert_adr + idx2) + 0],
                     m_->mesh_vert[3 * (vert_adr + idx2) + 1],
                     m_->mesh_vert[3 * (vert_adr + idx2) + 2]};

      uint32_t base_idx = static_cast<uint32_t>(vertices.size());
      vertices.push_back({v0[0], v0[1], v0[2]});
      vertices.push_back({v1[0], v1[1], v1[2]});
      vertices.push_back({v2[0], v2[1], v2[2]});
      faces.push_back({base_idx, base_idx + 1, base_idx + 2});

      if (norm_adr >= 0 && m_->mesh_facenormal != nullptr) {
        int nidx0 = m_->mesh_facenormal[3 * (face_adr + f) + 0];
        int nidx1 = m_->mesh_facenormal[3 * (face_adr + f) + 1];
        int nidx2 = m_->mesh_facenormal[3 * (face_adr + f) + 2];

        normals.push_back({m_->mesh_normal[3 * (norm_adr + nidx0) + 0],
                           m_->mesh_normal[3 * (norm_adr + nidx0) + 1],
                           m_->mesh_normal[3 * (norm_adr + nidx0) + 2]});
        normals.push_back({m_->mesh_normal[3 * (norm_adr + nidx1) + 0],
                           m_->mesh_normal[3 * (norm_adr + nidx1) + 1],
                           m_->mesh_normal[3 * (norm_adr + nidx1) + 2]});
        normals.push_back({m_->mesh_normal[3 * (norm_adr + nidx2) + 0],
                           m_->mesh_normal[3 * (norm_adr + nidx2) + 1],
                           m_->mesh_normal[3 * (norm_adr + nidx2) + 2]});
      } else {
        // Fallback: calcula normal da face
        float e1[3] = {v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
        float e2[3] = {v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
        float nx = e1[1] * e2[2] - e1[2] * e2[1];
        float ny = e1[2] * e2[0] - e1[0] * e2[2];
        float nz = e1[0] * e2[1] - e1[1] * e2[0];
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-6f) {
          nx /= len;
          ny /= len;
          nz /= len;
        } else {
          nz = 1.0f;
        }
        normals.push_back({nx, ny, nz});
        normals.push_back({nx, ny, nz});
        normals.push_back({nx, ny, nz});
      }
    }

    std::vector<rerun::Color> colors(vertices.size(), color);

    stream->log_static(entity_path, rerun::Mesh3D(vertices)
                                        .with_triangle_indices(faces)
                                        .with_vertex_normals(normals)
                                        .with_vertex_colors(colors));
    break;
  }
  default:
    break;
  }
}

void MujocoRerun::logModel() {
  if (!m_ || !rerun_client_ || !rerun_client_->ok()) {
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Publicando modelo geométrico estático no Rerun (raiz: '%s')...",
              entity_root_.c_str());

  std::string base = entity_root_;
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }

  std::string visual_root = base + "/visual_geometries";
  std::string collision_root = base + "/collision_geometries";
  std::string bodies_root = base + "/bodies";

  // Mapeia geoms por body
  std::vector<std::vector<int>> visual_geoms(m_->nbody);
  std::vector<std::vector<int>> collision_geoms(m_->nbody);

  for (int g = 0; g < m_->ngeom; ++g) {
    int b = m_->geom_bodyid[g];
    int contype = m_->geom_contype[g];
    int conaffinity = m_->geom_conaffinity[g];
    int group = m_->geom_group[g];

    // Convenção de visual geom: contype == 0 && conaffinity == 0 && group != 3
    if (contype == 0 && conaffinity == 0 && group != 3) {
      visual_geoms[b].push_back(g);
    } else {
      collision_geoms[b].push_back(g);
    }
  }

  for (int b = 0; b < m_->nbody; ++b) {
    const char *bname = mj_id2name(m_, mjOBJ_BODY, b);
    std::string body_name = bname ? bname : ("body_" + std::to_string(b));
    std::string body_path = bodies_root + "/" + body_name;
    std::string body_frame = "tf#" + body_path;

    const auto &v_list = visual_geoms[b];
    const auto &c_list = collision_geoms[b];

    const auto &target_list = (!v_list.empty()) ? v_list : c_list;
    for (int g : target_list) {
      const char *gname = mj_id2name(m_, mjOBJ_GEOM, g);
      std::string geom_name = gname ? gname : ("geom_" + std::to_string(g));
      std::string entity_path = visual_root + "/" + body_name + "/" + geom_name;
      logGeomWithFrame(entity_path, g, body_frame);
    }
  }

  // Publica o Pinhole padrão de câmera 2D para visualização de imagem e
  // bounding boxes
  float hfov_rad = 70.0f * static_cast<float>(M_PI) / 180.0f;
  float fx = (416.0f / 2.0f) / std::tan(hfov_rad / 2.0f);

  // Em Rerun / Rewire:
  // A bridge rewire mapeia tópicos 2D ligados a "camera_link" para o frame
  // "/camera_link#image". Conectamos o Pinhole de /camera_link para
  // /camera_link#image e posicionamos a imagem estática nesse frame.
  auto pinhole = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
                     fx, {416.0f, 416.0f})
                     .with_camera_xyz(rerun::components::ViewCoordinates::RDF)
                     .with_image_plane_distance(0.25f)
                     .with_parent_frame("/camera_link")
                     .with_child_frame("/camera_link#image");

  auto stream = rerun_client_->stream();

  stream->log_static("/perse/sensors/camera", pinhole);
  stream->log_static("/perse/sensors/camera",
                     rerun::CoordinateFrame("/camera_link#image"));

  // 2. Publica uma imagem estática de cor sólida (416x416 RGB) apenas uma vez
  //    para servir de plano 2D de fundo
  const uint32_t cam_w = 416;
  const uint32_t cam_h = 416;
  std::vector<uint8_t> solid_image(cam_w * cam_h * 3,
                                   20); // Cinza escuro sutil (20, 20, 20)
  auto static_img =
      rerun::archetypes::Image::from_rgb24(solid_image, {cam_w, cam_h});
  stream->log_static("/perse/sensors/camera", static_img);

  // Publica o estado inicial de transformações
  logData();
}

void MujocoRerun::logData() {
  if (!m_ || !d_ || !rerun_client_ || !rerun_client_->ok()) {
    return;
  }

  auto stream = rerun_client_->stream();

  std::string base = entity_root_;
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }
  std::string bodies_root = base + "/bodies";

  for (int b = 0; b < m_->nbody; ++b) {
    const char *bname = mj_id2name(m_, mjOBJ_BODY, b);
    std::string body_name = bname ? bname : ("body_" + std::to_string(b));
    std::string body_path = bodies_root + "/" + body_name;

    mjtNum *xpos = d_->xpos + 3 * b;
    mjtNum *xquat = d_->xquat + 4 * b; // wxyz

    rerun::components::Translation3D trans(static_cast<float>(xpos[0]),
                                           static_cast<float>(xpos[1]),
                                           static_cast<float>(xpos[2]));
    rerun::encodings::Quaternion quat = rerun::encodings::Quaternion::from_wxyz(
        static_cast<float>(xquat[0]), static_cast<float>(xquat[1]),
        static_cast<float>(xquat[2]), static_cast<float>(xquat[3]));

    stream->log(body_path,
                rerun::Transform3D::from_translation_rotation(trans, quat));
  }
}

void MujocoRerun::recognitionsCallback(
    const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
  if (!rerun_client_ || !rerun_client_->ok()) {
    return;
  }
  // Quando a lista de detecções estiver vazia (cones saíram do campo de visão),
  // emite um Clear FLAT na entidade para limpar imediatamente as bounding boxes
  // do Rerun
  if (msg->detections.empty()) {
    rerun_client_->stream()->log(recognitions_topic_,
                                 rerun::archetypes::Clear::FLAT);
  }
}

void MujocoRerun::ipcSyncLoop() {
  while (running_ && rclcpp::ok()) {
    int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sim_socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      close(sock_fd);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    std::string sub_req = "{\"cmd\": \"subscribe_state\"}\n";
    if (write(sock_fd, sub_req.c_str(), sub_req.size()) < 0) {
      close(sock_fd);
      continue;
    }

    RCLCPP_INFO(this->get_logger(),
                "Conectado ao socket de simulação '%s' para streaming Rerun.",
                sim_socket_path_.c_str());

    std::string buffer;
    char chunk[4096];
    bool model_logged = false;
    double min_interval =
        (publish_rate_hz_ > 0.0) ? (1.0 / publish_rate_hz_) : 0.0;
    double last_log_time = -1.0;

    while (running_ && rclcpp::ok()) {
      ssize_t bytes_read = read(sock_fd, chunk, sizeof(chunk) - 1);
      if (bytes_read <= 0) {
        break;
      }
      chunk[bytes_read] = '\0';
      buffer.append(chunk);

      size_t pos = 0;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);

        if (line.empty()) {
          continue;
        }

        try {
          auto msg = json::parse(line);
          if (msg.value("type", "") == "state_update") {
            std::lock_guard<std::mutex> lock(data_mutex_);
            d_->time = msg.value("sim_time", 0.0);
            if (msg.contains("qpos") && msg["qpos"].is_array()) {
              auto qpos = msg["qpos"].get<std::vector<double>>();
              int n = std::min<int>(m_->nq, qpos.size());
              for (int i = 0; i < n; ++i) {
                d_->qpos[i] = qpos[i];
              }
            }
            if (msg.contains("qvel") && msg["qvel"].is_array()) {
              auto qvel = msg["qvel"].get<std::vector<double>>();
              int n = std::min<int>(m_->nv, qvel.size());
              for (int i = 0; i < n; ++i) {
                d_->qvel[i] = qvel[i];
              }
            }

            mj_forward(m_, d_);

            if (!model_logged && rerun_client_ && rerun_client_->ok()) {
              logModel();
              model_logged = true;
            }

            if (min_interval <= 0.0 ||
                (d_->time - last_log_time) >= min_interval ||
                last_log_time < 0.0) {
              logData();
              last_log_time = d_->time;
            }
          }
        } catch (...) {
        }
      }
    }

    close(sock_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

} // namespace perse_mujoco
