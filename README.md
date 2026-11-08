# perse_mujoco

Pacote de simulação física baseada no **MuJoCo** para o robô **Perse**. Desenvolvido para substituir a antiga simulação do Webots (`perse_webots`), oferecendo desacoplamento completo de renderização (execução *headless* nativa ultrarrápida), simulação realista de dinâmicas de contato (suspensão prismática com mola e amortecedor, geometria Ackermann, atrito pneu-solo e bumper mecânico móvel), além de um **AI Harness** via **Model Context Protocol (MCP)** para interação bidirecional com agentes autônomos de IA.

---

## 🚀 Funcionalidades

- **Modelo Físico Completo do Robô Perse (`perse_robot.xml`):**
  - **Inércia e CoM Reais:** Chassi baseado na plataforma Traxxas Stampede ($3.5\,\text{kg}$) com matriz completa de inércia e centro de massa medido.
  - **Suspensão Independente nas 4 Rodas:** Juntas prismáticas lineares (`slide`) com rigidez $k = 1500\,\text{N/m}$ e amortecimento $c = 50\,\text{Ns/m}$, proporcionando assentamento suave e absorção de desníveis.
  - **Direção Ackermann Dianteira com Servos Realistas:** Juntas de esterçamento (`steer_fl_joint`, `steer_fr_joint`) com controle de posição via atuadores angulares ($\pm 0.8\,\text{rad} \approx \pm 45^\circ$), limite físico de torque (`forcerange = ±2.5 N.m`), inércia de redução interna (`armature = 0.01 kg.m²`) e amortecimento ajustado ($k_p = 25, k_v = 3$), eliminando solavancos e o salto da suspensão em manobras rápidas.
  - **Tração Traseira com Dinâmica Física Real:** Juntas de rotação contínua nos eixos traseiros com inércia de rotor refletida (`armature = 0.005 kg.m²`), saturação física de torque elétrico (`forcerange = ±1.5 N.m`) e atrito de borracha elástica, proporcionando resposta inercial de 1ª ordem sem solavancos ou picos artificiais de aceleração.
  - **Bumper Frontal Móvel:** Lâmina parachoque móvel montada sobre junta prismática linear com curso de $3\,\text{cm}$, mola de retorno de $2500\,\text{N/m}$ e batente rígido para detecção e absorção de colisões.
  - **Sensores Embarcados:** Sites calibrados para IMU (acelerômetro + giroscópio), LiDAR 2D (`RpLidarA2`), Câmera frontal e Sensor Indutivo.
  - **Malhas 3D Alinhadas (STLs):** Chassi mecânico (`locomocao`), tampa superior (`tampa`), rodas Traxxas, bumper, LiDAR e câmera.
- **Integração Total com o ROS 2 (Sensores & Atuadores):**
  - **Relógio de Simulação (`/clock`):** Publicação de `rosgraph_msgs/msg/Clock` a cada passo físico, com suporte completo a `use_sim_time: true`.
  - **IMU Completa (`/perse/sensors/imu`):** Acelerômetro e giroscópio acoplados ao `base_link` com matrizes de covariância.
  - **Odometria de Rodas (`/perse/sensors/encoder_fusion/distance`):** Leitura angular quantizada (resolução de $1\,\text{mm}$) com paridade total ao robô real e Webots.
  - **Bumper de Impacto (`/perse/sensors/bumper`):** Detecção de contato por contração da junta linear ($q < -0.005\,\text{m}$) e colisões ativas.
  - **Sensor Indutivo (`/perse/sensors/inductive_sensor`):** Detecção de proximidade 2D em relação às placas metálicas ($1 \times 1\,\text{m}$) associadas aos cones de missão.
  - **LiDAR 2D 360° (`/perse/sensors/lidar`):** Varredura analítica de 360 feixes via raycasting `mj_ray()` em loop C++ a 10 Hz.
  - **Visão Sintética Ground-Truth (`/perse/sensors/camera/recognitions`):** Projeção analítica dos cones ativos no frustum da câmera frontal ($416 \times 416$), com frustum culling e teste de oclusão via raycast `mj_ray()`, consumível diretamente pelo `ControlVision`.
  - **Ground Truth Odometry (`/perse/simulation/odom_gt`):** Pose e velocidades reais extraídas diretamente da raiz livre do robô.
  - **Atuação Ackermann e Velocidade (`topics.cmd_vel`):** Cinemática Ackermann automática para os servos dianteiros e controle de cruzeiro nas rodas traseiras com mapeamento calibrado para comandos normalizados (0 a 1) do robô real, consumindo limites de velocidade (`properties.limits.speed.forward` e `properties.limits.speed.backward`), limites mecânicos e geometria diretamente de `perse_properties.yaml` e tópicos de `topics.yaml`.
  - **Haste de LED (`topics.led`):** Controle de acionamento do sinalizador de missão.
- **Simulação Headless Determinística:** O loop de física roda em processo ROS 2 desacoplado de interface gráfica, com publicação de `/clock` e taxa de 500 Hz com integrador implícito `implicitfast`.
- **Ponte IPC de Alta Performance:** Comunicação via Unix Domain Socket (`/tmp/perse_mujoco.sock`) permitindo controle de tempo e coleta de telemetria sem interferir na estabilidade do motor de física.
- **Controle Temporal Híbrido:**
  - `Free Run`: Execução contínua sincronizada com o relógio ROS 2.
  - `Step Mode`: Execução passo-a-passo sob demanda (`sim_step(N)`), ideal para testes automatizados, depuração e IA.
- **AI Harness (MCP):** Servidor MCP integrado (`perse_mujoco_mcp`) baseado no SDK oficial da Anthropic (`MCPServer`), expondo ferramentas de inspeção, telemetria, controle de câmera e captura de screenshots.
- **Suite de Testes Automatizados:** Testes físicos (`test/test_perse_model.py`), renderizador offscreen (`test/test_renderer.py`) e testes de malha fechada ROS 2 (`test/test_ros_bridge.py`).

---

## 📁 Estrutura do Pacote

```
perse_mujoco/
├── CMakeLists.txt                  # Configuração CMake (find_package / FetchContent, nós e bibliotecas)
├── package.xml                     # Metadados e dependências ROS 2 Humble
├── README.md                       # Documentação principal do pacote
├── SPEC.md                         # Especificação técnica completa da simulação
├── AI_HARNESS_SPEC.md              # Especificação de arquitetura do harness de IA
├── config/                         # Parâmetros de simulação e arenas
│   └── perse_mujoco.yaml           # Configurações do nó da simulação (taxas, socket IPC)
├── include/perse_mujoco/           # Headers C++ (exclusivamente declarações)
│   ├── ActuatorBridge.hpp          # Subscrições cmd_vel, led_rod e cinemática Ackermann
│   ├── MujocoIpcBridge.hpp         # Interface da ponte IPC via Unix Domain Socket
│   ├── MujocoViewer.hpp            # Interface do visualizador nativo GLFW
│   ├── PerseMujocoNode.hpp         # Nó principal de simulação física e loop ROS 2
│   ├── SensorBridge.hpp            # Conversores MuJoCo -> Tópicos ROS 2 (IMU, Encoders, Bumper, Lidar, Odom)
│   └── SyntheticVision.hpp         # Projeção analítica de cones, frustum culling e oclusão via raycast
├── src/                            # Implementações e nós C++
│   ├── lib/                        # Implementações de classes e componentes do simulador
│   │   ├── ActuatorBridge.cpp      # Mapeamento Ackermann e velocidades nas rodas
│   │   ├── MujocoIpcBridge.cpp     # Implementação do servidor socket e comandos JSON
│   │   ├── MujocoViewer.cpp        # Visualizador nativo GLFW com suporte a screenshot e IPC
│   │   ├── PerseMujocoNode.cpp     # Implementação do nó central e avanço determinístico de física
│   │   ├── SensorBridge.cpp        # Extração de sensores e publicadores ROS 2
│   │   └── SyntheticVision.cpp     # Projeção geométrica 2D e raycasting de linha de visada
│   ├── perse_mujoco_node.cpp       # Entrypoint main() do nó de simulação
│   └── perse_mujoco_viewer.cpp     # Entrypoint main() do nó visualizador
├── model/                          # Modelos MJCF e malhas 3D
│   ├── perse_robot.xml.xacro       # Template paramétrico Xacro sincronizado com perse_properties.yaml
│   ├── perse_robot.xml             # Modelo MJCF gerado automaticamente no build via Xacro
│   ├── default_scene.xml           # Cena padrão mínima (apenas robô e chão para fallback autônomo)
│   ├── generated_scene.xml         # Cenário montado dinamicamente em runtime pelo map_generator_node
│   └── meshes/                     # Malhas 3D processadas (OBJs com normais suaves e STLs)
│       ├── cone_body.obj           # Corpo do cone de trânsito em laranja vibrante
│       ├── cone_stripes.obj        # Listras brancas refletivas do cone
│       ├── traffic_cone.obj        # Modelo original do cone
│       ├── locomocao_crease.obj    # Chassi mecânico com normais de vértice suaves
│       ├── tampa_crease.obj        # Tampa/carenagem superior com crease-shading
│       ├── bumper_base_fixed.obj   # Suporte de montagem fixo aparafusado no chassi
│       ├── bumper_moving.obj       # Botão móvel do parachoque (aletas telescópicas + barra)
│       ├── wheel_aligned.stl       # Roda alinhada no eixo Y
│       ├── rplidar_a2.stl          # Malha oficial do Slamtec RpLidar A2 (Webots)
│       └── camera_centered.stl     # Malha da câmera frontal
├── launch/                         # Launch files em formato XML
│   ├── sim.launch                  # Launch principal MuJoCo (nó de física, TFs estáticos e GUI opcional)
│   ├── sim_test.launch             # Inicialização básica de teste do nó de física
│   └── viewer.launch               # Inicialização da janela interativa GLFW
├── test/                           # Testes automatizados e cenas de teste
│   ├── model/                      # Modelos e cenários MJCF de teste
│   │   └── test_scene.xml          # Cena de validação física com obstáculos, chapas e cones de teste
│   ├── cpp/                        # Nós e utilitários C++ para testes
│   │   ├── TestSimNode.hpp
│   │   └── TestSimNode.cpp
│   ├── test_perse_model.py         # Testes de integridade física (suspensão, direção, tração e bumper)
│   ├── test_renderer.py            # Renderizador offscreen para testes visuais
│   └── test_ros_bridge.py          # Testes de integração ROS 2 (tópicos, sensores, visão e resposta dinâmica)
└── python/perse_mujoco/            # Módulos Python
    ├── map_generator_node.py       # Nó ROS 2 de geração dinâmica de mapas (serviço Trigger)
    └── mcp_server/                 # Servidor e cliente MCP
        ├── ipc_client.py           # Cliente IPC Unix Domain Socket
        └── server.py               # Servidor MCP stdio (ferramentas de IA)
```

---

## 🛠️ Compilação e Execução

Dentro do container de desenvolvimento (`perse_container`):

```bash
# Compilar os pacotes
colcon build --symlink-install --packages-select perse_lib perse_localization perse_base perse_bringup perse_mujoco

# Carregar o workspace
source install/setup.bash

# 1. Rodar a simulação MuJoCo com interface gráfica nativa GLFW e TFs:
ros2 launch perse_mujoco sim.launch gui:=true

# 2. Rodar a simulação MuJoCo em modo headless (sem janela):
ros2 launch perse_mujoco sim.launch gui:=false

# 3. Rodar a stack completa do robô Perse usando o novo simulador MuJoCo:
ros2 launch perse_bringup perse.launch sim:=true simulator:=mujoco

# (Opcional) Rodar com o simulador legado Webots:
ros2 launch perse_bringup perse.launch sim:=true simulator:=webots
```

> **Nota:** O visualizador `perse_mujoco_viewer` é completamente desacoplado do nó de simulação. Você pode fechar e reabrir a simulação (`sim.launch`) à vontade, e a janela do visualizador permanecerá aberta, reconectando-se automaticamente assim que a simulação reiniciar!

### 🧪 Execução da Suite de Testes

1. **Testes de Integração ROS 2 (Publicação de sensores, atuação e resposta física):**
```bash
# Com a simulação rodando em background (ex.: ros2 launch perse_mujoco sim.launch gui:=false):
python3 src/perse_mujoco/test/test_ros_bridge.py
```

2. **Testes Unitários de Dinâmica Física do Modelo MuJoCo:**
```bash
python3 src/perse_mujoco/test/test_perse_model.py
```

### 🖱️ Controles e Gestos Interativos na Janela (MuJoCo Studio Standard):
| Ação | Comando do Mouse / Teclado |
|---|---|
| **Rotacionar / Orbitar Câmera** | Clique e arraste com o **Botão Esquerdo** |
| **Transladar Câmera (Pan)** | Clique e arraste com o **Botão Direito** (desativado quando no modo de seguir) |
| **Zoom** | **Scroll** do mouse ou arraste com **Botão do Meio** |
| **Seguir o Robô Perse (Webots-style)** | Pressione a tecla **`F`** (ativa/desativa). No modo ativo, o mouse orbita ao redor do robô |
| **Reiniciar Simulação** | Pressione **`Ctrl + R`** (restaura posições, tempo e zera atuadores) |
| **Centralizar Ponto Focal de Órbita** | **Duplo clique** (com botão esquerdo ou direito) em qualquer ponto ou objeto |
| **Puxar / Aplicar Força no Objeto ("Corda")** | Segure **Ctrl** + clique e arraste com o **Botão Direito** sobre o objeto |

---

## 🤖 Como Configurar e Rodar o MCP com o Antigravity

O servidor MCP permite que o agente **Antigravity** interaja diretamente com a simulação ativa e com o visualizador, pausando o tempo, executando passos determinísticos, inspecionando corpos, movendo a câmera e capturando screenshots em tempo real.

### 1. Configurar o Antigravity

Adicione o servidor `perse_mujoco` no arquivo global de configuração de MCP do Antigravity em `~/.gemini/config/mcp_config.json`:

```json
{
  "mcpServers": {
    "perse_mujoco": {
      "command": "docker",
      "args": [
        "exec",
        "-i",
        "-e",
        "PYTHONPATH=/home/thor/perse_ws/src/perse_mujoco/python",
        "perse_container",
        "python3",
        "/home/thor/perse_ws/src/perse_mujoco/python/perse_mujoco/mcp_server/server.py"
      ]
    }
  }
}
```

### 2. Ferramentas Disponíveis para o Agente

#### Simulação Física:
| Ferramenta | Descrição |
|---|---|
| `sim_get_status` | Retorna o status atual (tempo simulado, modo `free_run`/`step_mode`, corpos e juntas). |
| `sim_step` | Avança $N$ passos físicos no MuJoCo e pausa o tempo (determinístico). |
| `sim_pause` | Pausa imediatamente o loop de simulação. |
| `sim_resume` | Retoma a execução livre contínua em tempo real. |
| `sim_set_speed` | Ajusta o multiplicador de velocidade da simulação física (ex.: 0.5x, 1.0x, 2.0x). |
| `sim_reset` | Reseta a simulação para o estado inicial ($t = 0$). |
| `sim_get_bodies` | Retorna posições e rotações 3D (quaternions) de todos os corpos. |
| `sim_get_joints` | Retorna valores de deslocamento/ângulo (`qpos`) e velocidade (`qvel`) das juntas. |
| `sim_apply_force` | Aplica vetor de força $[F_x, F_y, F_z]$ e/ou torque $[T_x, T_y, T_z]$ em um corpo. |

#### Visualizador Nativo:
| Ferramenta | Descrição |
|---|---|
| `viewer_get_status` | Informa se o visualizador está conectado à simulação, parâmetros da câmera atual e flags visuais de depuração. |
| `viewer_set_camera` | Move a câmera interativa (ponto de foco `lookat [x,y,z]`, distância, azimute e elevação). |
| `viewer_focus_body` | Centraliza e foca a câmera automaticamente sobre qualquer corpo (`chassis`, `bumper`, `wheel_fl`, etc.). |
| `viewer_set_option` | Ativa ou desativa modos de depuração visual (`contacts`, `joints`, `com`, `inertia`, `wireframe`, `shadow`, `transparent`). |
| `viewer_capture_screenshot` | Captura um screenshot em alta fidelidade diretamente da viewport do visualizador em tempo real. |
