#include "perse_mujoco/MujocoViewer.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace perse_mujoco {

// Estado de interação do mouse compatível com o MuJoCo Studio
static bool button_left = false;
static bool button_middle = false;
static bool button_right = false;
static double lastx = 0;
static double lasty = 0;

// Estado de perturbação e seleção controlado pelo usuário
static bool pert_dragging = false;
static int saved_body_id = 0;
static mjtNum saved_localpos[3] = {0, 0, 0};

static double last_click_time = 0;

static void mouse_button(GLFWwindow *window, int button, int act, int /*mods*/) {
  auto *viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (!viewer) return;

  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  glfwGetCursorPos(window, &lastx, &lasty);

  bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                   glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

  // Duplo clique com o botão esquerdo ou direito (sem ctrl): centraliza a câmera (lookat) no ponto clicado
  if (act == GLFW_PRESS && !mod_ctrl) {
    double now = glfwGetTime();
    if (now - last_click_time < 0.25) {
      int width, height;
      glfwGetFramebufferSize(window, &width, &height);
      mjtNum selpnt[3];
      int selgeom = -1, selflex = -1, selskin = -1;

      std::lock_guard<std::mutex> lock(viewer->getDataMutex());
      int selbody = mjv_select(viewer->getModel(), viewer->getData(), viewer->getOption(),
                               static_cast<mjtNum>(width) / height,
                               lastx / width, (height - lasty) / height,
                               viewer->getScene(), selpnt, &selgeom, &selflex, &selskin);
      if (selbody >= 0) {
        // Recentraliza o ponto focal de órbita da câmera
        viewer->getCamera()->lookat[0] = selpnt[0];
        viewer->getCamera()->lookat[1] = selpnt[1];
        viewer->getCamera()->lookat[2] = selpnt[2];
      }
    }
    last_click_time = now;
  }

  // Se soltar o clique direito, zera a perturbação e força aplicada
  if (act == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_RIGHT) {
    std::lock_guard<std::mutex> lock(viewer->getDataMutex());
    if (pert_dragging && saved_body_id > 0) {
      const char *bname = mj_id2name(viewer->getModel(), mjOBJ_BODY, saved_body_id);
      std::string body_name = bname ? bname : "body_" + std::to_string(saved_body_id);
      viewer->sendSimCommand({
          {"cmd", "apply_force"},
          {"body", body_name},
          {"force", {0.0, 0.0, 0.0}}
      });
    }
    pert_dragging = false;
    saved_body_id = 0;
    viewer->getPerturb()->select = 0;
    viewer->getPerturb()->active = 0;
    mjv_initPerturb(viewer->getModel(), viewer->getData(), viewer->getScene(), viewer->getPerturb());
    return;
  }

  // Só inicia seleção e perturbação se for Ctrl + Botão Direito
  if (act == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_RIGHT && mod_ctrl) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    mjtNum selpnt[3];
    int selgeom = -1;
    int selflex = -1;
    int selskin = -1;

    std::lock_guard<std::mutex> lock(viewer->getDataMutex());
    int selbody = mjv_select(viewer->getModel(), viewer->getData(), viewer->getOption(),
                             static_cast<mjtNum>(width) / height,
                             xpos / width, (height - ypos) / height,
                             viewer->getScene(), selpnt, &selgeom, &selflex, &selskin);

    if (selbody > 0) {
      saved_body_id = selbody;
      pert_dragging = true;

      viewer->getPerturb()->select = selbody;
      viewer->getPerturb()->active = mjPERT_TRANSLATE;
      viewer->getPerturb()->refpos[0] = selpnt[0];
      viewer->getPerturb()->refpos[1] = selpnt[1];
      viewer->getPerturb()->refpos[2] = selpnt[2];
      viewer->getPerturb()->refselpos[0] = selpnt[0];
      viewer->getPerturb()->refselpos[1] = selpnt[1];
      viewer->getPerturb()->refselpos[2] = selpnt[2];

      // Converte o ponto global de seleção para o referencial local do corpo:
      // localpos = R^T * (selpnt - body_xpos)
      mjtNum diff[3];
      mju_sub3(diff, selpnt, viewer->getData()->xpos + 3 * selbody);
      mju_mulMatTVec3(saved_localpos, viewer->getData()->xmat + 9 * selbody, diff);
      mju_copy3(viewer->getPerturb()->localpos, saved_localpos);

      mjv_initPerturb(viewer->getModel(), viewer->getData(), viewer->getScene(), viewer->getPerturb());
    }
  } else if (act == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_RIGHT && !mod_ctrl) {
    // Clique direito simples: NÃO ativa perturbação nem seleciona corpo (remove highlight e indicador)
    std::lock_guard<std::mutex> lock(viewer->getDataMutex());
    pert_dragging = false;
    saved_body_id = 0;
    viewer->getPerturb()->select = 0;
    viewer->getPerturb()->active = 0;
  }
}

static void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  auto *viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (!viewer) return;

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  if (!button_left && !button_middle && !button_right) return;

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                   glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // Arrastar com Ctrl + Botão Direito: move o ponto alvo de perturbação da mola/corda
  if (mod_ctrl && button_right && pert_dragging && saved_body_id > 0) {
    std::lock_guard<std::mutex> lock(viewer->getDataMutex());
    mjv_movePerturb(viewer->getModel(), viewer->getData(), action,
                    dx / height, dy / height,
                    viewer->getScene(), viewer->getPerturb());
    viewer->applySpringForce();
  } else {
    // Modo padrão de câmera do MuJoCo Studio:
    // Botão esquerdo: rotação de câmera
    // Botão direito: pan/translação de câmera
    // Botão do meio ou scroll: zoom
    std::lock_guard<std::mutex> lock(viewer->getDataMutex());
    mjv_moveCamera(viewer->getModel(), action, dx / height, dy / height,
                   viewer->getCamera());
  }
}

static void scroll(GLFWwindow *window, double /*xoffset*/, double yoffset) {
  auto *viewer = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (!viewer) return;

  std::lock_guard<std::mutex> lock(viewer->getDataMutex());
  mjv_moveCamera(viewer->getModel(), mjMOUSE_ZOOM, 0, 0.05 * yoffset,
                 viewer->getCamera());
}

MujocoViewer::MujocoViewer(const std::string &model_path,
                           const std::string &sim_socket_path,
                           const std::string &viewer_socket_path)
    : model_path_(model_path), sim_socket_path_(sim_socket_path),
      viewer_socket_path_(viewer_socket_path) {}

MujocoViewer::~MujocoViewer() { stop(); }

bool MujocoViewer::init() {
  char error[1000] = "Não foi possível carregar o arquivo XML";
  m_ = mj_loadXML(model_path_.c_str(), nullptr, error, 1000);
  if (!m_) {
    std::cerr << "Erro ao carregar modelo no visualizador: " << error << std::endl;
    return false;
  }

  d_ = mj_makeData(m_);
  if (!d_) {
    std::cerr << "Erro ao alocar mjData no visualizador." << std::endl;
    return false;
  }

  if (!glfwInit()) {
    std::cerr << "Falha ao inicializar GLFW." << std::endl;
    return false;
  }

  window_ = glfwCreateWindow(1200, 800, "Perse - MuJoCo Native Viewer", nullptr, nullptr);
  if (!window_) {
    std::cerr << "Falha ao criar janela GLFW (verifique se DISPLAY está configurado)." << std::endl;
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);

  glfwSetMouseButtonCallback(window_, mouse_button);
  glfwSetCursorPosCallback(window_, mouse_move);
  glfwSetScrollCallback(window_, scroll);

  // Inicializa estruturas de renderização do MuJoCo
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  // Oculta por padrão geoms de colisão e sites de medição (grupo 3) para evitar sobreposição/z-fighting
  opt_.geomgroup[3] = 0;
  opt_.sitegroup[3] = 0;
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_defaultPerturb(&pert_);

  mjv_makeScene(m_, &scn_, 2000);
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);

  // Posição inicial da câmera
  cam_.distance = 3.5;
  cam_.azimuth = 135;
  cam_.elevation = -25;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.2;

  running_ = true;

  // Thread de sincronização IPC contínua com a simulação
  sync_thread_ = std::thread(&MujocoViewer::ipcSyncLoop, this);

  // Thread de controle MCP do visualizador (câmera, capturas, info)
  control_thread_ = std::thread(&MujocoViewer::controlServerLoop, this);

  return true;
}

void MujocoViewer::run() {
  while (running_ && !glfwWindowShouldClose(window_)) {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      // Se estiver arrastando com Ctrl + Botão Direito, atualiza e envia a força continuamente
      applySpringForce();
      // Aplica perturbação visual se houver corpo selecionado
      mjv_updateScene(m_, d_, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
    }

    mjr_render(viewport, &scn_, &con_);

    // Overlay de status e instruções na janela
    if (!connected_to_sim_) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                  "Simulacao Desconectada", "Aguardando conexao com perse_mujoco_node...",
                  &con_);
    } else {
      mjr_overlay(mjFONT_SHADOW, mjGRID_BOTTOMLEFT, viewport,
                  "Mouse: Botao Esq: Rotacao | Botao Dir: Pan | Scroll: Zoom | Duplo Clique: Centralizar Ponto Focal",
                  "Ctrl + Botao Dir: Aplicar Forca/Perturbacao no Objeto",
                  &con_);
    }

    // Se houver pedido de screenshot pendente, executa aqui na thread GLFW/OpenGL ativa
    {
      std::unique_lock<std::mutex> req_lock(screenshot_req_.mtx);
      if (screenshot_req_.pending) {
        mjrRect shot_viewport = {0, 0, screenshot_req_.width, screenshot_req_.height};
        if (shot_viewport.width <= 0 || shot_viewport.height <= 0) {
          shot_viewport = viewport;
        }

        std::vector<unsigned char> rgb(3 * shot_viewport.width * shot_viewport.height);
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          mjv_updateScene(m_, d_, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
        }
        mjr_render(shot_viewport, &scn_, &con_);
        mjr_readPixels(rgb.data(), nullptr, shot_viewport, &con_);

        std::string ppm_path = screenshot_req_.path;
        if (ppm_path.find(".png") != std::string::npos) {
          ppm_path.replace(ppm_path.find(".png"), 4, ".ppm");
        }

        std::ofstream ofs(ppm_path, std::ios::binary);
        if (ofs) {
          ofs << "P6\n" << shot_viewport.width << " " << shot_viewport.height << "\n255\n";
          for (int y = shot_viewport.height - 1; y >= 0; --y) {
            ofs.write(reinterpret_cast<char *>(&rgb[3 * shot_viewport.width * y]), 3 * shot_viewport.width);
          }
          ofs.close();

          std::string cmd = "python3 -c \"import cv2; img=cv2.imread('" + ppm_path + "'); cv2.imwrite('" + screenshot_req_.path + "', img)\" 2>/dev/null && rm -f " + ppm_path;
          int ret = system(cmd.c_str());
          if (ret != 0) {
            std::string fallback_cmd = "convert " + ppm_path + " " + screenshot_req_.path + " 2>/dev/null && rm -f " + ppm_path;
            (void)system(fallback_cmd.c_str());
          }
          screenshot_req_.success = true;
        } else {
          screenshot_req_.success = false;
        }

        screenshot_req_.pending = false;
        screenshot_req_.cv.notify_all();
      }
    }

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  running_ = false;
}

void MujocoViewer::stop() {
  if (running_) {
    running_ = false;
  }
  if (control_server_fd_ >= 0) {
    shutdown(control_server_fd_, SHUT_RDWR);
    close(control_server_fd_);
    control_server_fd_ = -1;
  }
  unlink(viewer_socket_path_.c_str());

  if (sync_thread_.joinable()) sync_thread_.join();
  if (control_thread_.joinable()) control_thread_.join();

  if (window_) {
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    glfwDestroyWindow(window_);
    glfwTerminate();
    window_ = nullptr;
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

void MujocoViewer::applySpringForce() {
  if (!pert_dragging || saved_body_id <= 0) return;

  // Ponto atual no referencial global onde a corda está presa no corpo:
  // selpoint = body_xpos + body_xmat * saved_localpos
  mjtNum selpoint[3];
  mju_mulMatVec3(selpoint, d_->xmat + 9 * saved_body_id, saved_localpos);
  mju_addTo3(selpoint, d_->xpos + 3 * saved_body_id);

  // Atualiza refselpos na perturbação para desenhar a linha exatamente a partir do ponto de contato atual
  pert_.refselpos[0] = selpoint[0];
  pert_.refselpos[1] = selpoint[1];
  pert_.refselpos[2] = selpoint[2];

  // Vetor de deslocamento do ponto alvo do mouse em relação ao ponto fixado na peça
  mjtNum delta[3] = {
      pert_.refpos[0] - selpoint[0],
      pert_.refpos[1] - selpoint[1],
      pert_.refpos[2] - selpoint[2]
  };

  // Rigidez da mola e amortecimento de velocidade no ponto de contato
  double k_spring = 35.0; // N/m
  double c_damping = 4.0; // N*s/m

  // Velocidade linear no ponto de aplicação: v_p = v_com + omega x (p - xipos)
  mjtNum pntvel[3] = {0, 0, 0};
  if (d_->cvel) {
    const mjtNum *omega = d_->cvel + 6 * saved_body_id;
    const mjtNum *v_com = d_->cvel + 6 * saved_body_id + 3;
    mjtNum r_com[3];
    mju_sub3(r_com, selpoint, d_->xipos + 3 * saved_body_id);
    mjtNum v_rot[3];
    mju_cross(v_rot, omega, r_com);
    mju_add3(pntvel, v_com, v_rot);
  }

  std::vector<double> force = {
      k_spring * delta[0] - c_damping * pntvel[0],
      k_spring * delta[1] - c_damping * pntvel[1],
      k_spring * delta[2] - c_damping * pntvel[2]
  };

  // Satura a força máxima para estabilidade física
  double max_force = 20.0; // Newtons
  double force_norm = std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
  if (force_norm > max_force) {
    force[0] = (force[0] / force_norm) * max_force;
    force[1] = (force[1] / force_norm) * max_force;
    force[2] = (force[2] / force_norm) * max_force;
  }

  const char *bname = mj_id2name(m_, mjOBJ_BODY, saved_body_id);
  std::string body_name = bname ? bname : "body_" + std::to_string(saved_body_id);
  std::vector<double> point = {selpoint[0], selpoint[1], selpoint[2]};

  sendSimCommand({
      {"cmd", "apply_force"},
      {"body", body_name},
      {"force", force},
      {"point", point}
  });
}

void MujocoViewer::sendSimCommand(const json &cmd) {
  int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (sock_fd < 0) return;

  struct sockaddr_un addr {};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, sim_socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (connect(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
    std::string msg = cmd.dump() + "\n";
    (void)write(sock_fd, msg.c_str(), msg.size());
  }
  close(sock_fd);
}

void MujocoViewer::ipcSyncLoop() {
  while (running_) {
    int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, sim_socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(sock_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      close(sock_fd);
      connected_to_sim_ = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    std::string sub_req = "{\"cmd\": \"subscribe_state\"}\n";
    if (write(sock_fd, sub_req.c_str(), sub_req.size()) < 0) {
      close(sock_fd);
      connected_to_sim_ = false;
      continue;
    }

    connected_to_sim_ = true;
    std::string buffer;
    char chunk[4096];

    while (running_) {
      ssize_t bytes_read = read(sock_fd, chunk, sizeof(chunk) - 1);
      if (bytes_read <= 0) break;
      chunk[bytes_read] = '\0';
      buffer.append(chunk);

      size_t pos = 0;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        buffer.erase(0, pos + 1);

        if (line.empty()) continue;

        try {
          auto msg = json::parse(line);
          if (msg.value("type", "") == "state_update") {
            std::lock_guard<std::mutex> lock(data_mutex_);
            d_->time = msg.value("sim_time", 0.0);
            if (msg.contains("qpos") && msg["qpos"].is_array()) {
              auto qpos = msg["qpos"].get<std::vector<double>>();
              int n = std::min<int>(m_->nq, qpos.size());
              for (int i = 0; i < n; ++i) d_->qpos[i] = qpos[i];
            }
            if (msg.contains("qvel") && msg["qvel"].is_array()) {
              auto qvel = msg["qvel"].get<std::vector<double>>();
              int n = std::min<int>(m_->nv, qvel.size());
              for (int i = 0; i < n; ++i) d_->qvel[i] = qvel[i];
            }
            mj_forward(m_, d_);
          }
        } catch (...) {}
      }
    }

    close(sock_fd);
    connected_to_sim_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void MujocoViewer::setCamera(double lookat[3], double distance, double azimuth, double elevation) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  cam_.lookat[0] = lookat[0];
  cam_.lookat[1] = lookat[1];
  cam_.lookat[2] = lookat[2];
  cam_.distance = distance;
  cam_.azimuth = azimuth;
  cam_.elevation = elevation;
}

bool MujocoViewer::captureFrame(const std::string &output_png_path, int width, int height) {
  std::unique_lock<std::mutex> req_lock(screenshot_req_.mtx);
  screenshot_req_.path = output_png_path;
  screenshot_req_.width = width;
  screenshot_req_.height = height;
  screenshot_req_.success = false;
  screenshot_req_.pending = true;

  // Aguarda a thread principal do GLFW processar a captura (timeout de 2 segundos)
  bool finished = screenshot_req_.cv.wait_for(req_lock, std::chrono::seconds(2), [this] {
    return !screenshot_req_.pending;
  });

  return finished && screenshot_req_.success;
}

void MujocoViewer::controlServerLoop() {
  unlink(viewer_socket_path_.c_str());

  control_server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (control_server_fd_ < 0) return;

  struct sockaddr_un addr {};
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, viewer_socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (bind(control_server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
      listen(control_server_fd_, 5) < 0) {
    close(control_server_fd_);
    control_server_fd_ = -1;
    return;
  }

  while (running_) {
    int client_fd = accept(control_server_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      if (!running_) break;
      continue;
    }
    handleControlClient(client_fd);
    close(client_fd);
  }
}

void MujocoViewer::handleControlClient(int client_fd) {
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
        json resp = dispatchControlCommand(req);
        std::string resp_str = resp.dump() + "\n";
        (void)write(client_fd, resp_str.c_str(), resp_str.size());
      } catch (const std::exception &e) {
        json err = {{"status", "error"}, {"message", e.what()}};
        std::string err_str = err.dump() + "\n";
        (void)write(client_fd, err_str.c_str(), err_str.size());
      }
    }
  }
}

bool MujocoViewer::focusOnBody(const std::string &body_name, double distance) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  int body_id = mj_name2id(m_, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0) return false;

  cam_.lookat[0] = d_->xpos[3 * body_id + 0];
  cam_.lookat[1] = d_->xpos[3 * body_id + 1];
  cam_.lookat[2] = d_->xpos[3 * body_id + 2];
  if (distance > 0.0) {
    cam_.distance = distance;
  }
  return true;
}

bool MujocoViewer::setVisualizationOption(const std::string &option, bool enabled) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  int val = enabled ? 1 : 0;

  if (option == "wireframe") {
    opt_.flags[mjRND_WIREFRAME] = val;
  } else if (option == "shadow") {
    opt_.flags[mjRND_SHADOW] = val;
  } else if (option == "joints") {
    opt_.flags[mjVIS_JOINT] = val;
  } else if (option == "contacts") {
    opt_.flags[mjVIS_CONTACTPOINT] = val;
    opt_.flags[mjVIS_CONTACTFORCE] = val;
  } else if (option == "com") {
    opt_.flags[mjVIS_COM] = val;
  } else if (option == "inertia") {
    opt_.flags[mjVIS_INERTIA] = val;
  } else if (option == "transparent") {
    opt_.flags[mjVIS_TRANSPARENT] = val;
  } else if (option == "collision" || option == "collisions") {
    opt_.geomgroup[3] = val;
  } else {
    return false;
  }
  return true;
}

json MujocoViewer::dispatchControlCommand(const json &req) {
  std::string cmd = req.value("cmd", "");

  if (cmd == "get_viewer_status") {
    return {
        {"status", "ok"},
        {"connected_to_sim", connected_to_sim_.load()},
        {"sim_time", d_->time},
        {"camera", {
            {"distance", cam_.distance},
            {"azimuth", cam_.azimuth},
            {"elevation", cam_.elevation},
            {"lookat", {cam_.lookat[0], cam_.lookat[1], cam_.lookat[2]}}
        }},
        {"options", {
            {"wireframe", opt_.flags[mjRND_WIREFRAME] != 0},
            {"shadow", opt_.flags[mjRND_SHADOW] != 0},
            {"joints", opt_.flags[mjVIS_JOINT] != 0},
            {"contacts", opt_.flags[mjVIS_CONTACTPOINT] != 0},
            {"com", opt_.flags[mjVIS_COM] != 0},
            {"inertia", opt_.flags[mjVIS_INERTIA] != 0},
            {"transparent", opt_.flags[mjVIS_TRANSPARENT] != 0},
            {"collisions", opt_.geomgroup[3] != 0}
        }}
    };
  }

  if (cmd == "set_camera") {
    double lookat[3] = {
        req.value("lookat", std::vector<double>{cam_.lookat[0], cam_.lookat[1], cam_.lookat[2]})[0],
        req.value("lookat", std::vector<double>{cam_.lookat[0], cam_.lookat[1], cam_.lookat[2]})[1],
        req.value("lookat", std::vector<double>{cam_.lookat[0], cam_.lookat[1], cam_.lookat[2]})[2]
    };
    double distance = req.value("distance", cam_.distance);
    double azimuth = req.value("azimuth", cam_.azimuth);
    double elevation = req.value("elevation", cam_.elevation);

    setCamera(lookat, distance, azimuth, elevation);
    return {{"status", "ok"}, {"camera_updated", true}};
  }

  if (cmd == "focus_body") {
    std::string body = req.value("body", "");
    double distance = req.value("distance", 2.0);
    bool ok = focusOnBody(body, distance);
    if (!ok) {
      return {{"status", "error"}, {"message", "Body not found: " + body}};
    }
    return {{"status", "ok"}, {"focused_on", body}};
  }

  if (cmd == "set_option") {
    std::string option = req.value("option", "");
    bool enabled = req.value("enabled", true);
    bool ok = setVisualizationOption(option, enabled);
    if (!ok) {
      return {{"status", "error"}, {"message", "Invalid visualization option: " + option}};
    }
    return {{"status", "ok"}, {"option", option}, {"enabled", enabled}};
  }

  if (cmd == "capture_screenshot") {
    std::string path = req.value("path", "/tmp/perse_screenshot.png");
    int width = req.value("width", 1200);
    int height = req.value("height", 800);
    bool ok = captureFrame(path, width, height);
    return {{"status", ok ? "ok" : "error"}, {"path", path}};
  }

  return {{"status", "error"}, {"message", "Comando desconhecido: " + cmd}};
}

} // namespace perse_mujoco
