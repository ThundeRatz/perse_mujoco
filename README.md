# perse_mujoco

Pacote de simulação física baseada no **MuJoCo** para o robô **Perse**. Desenvolvido para substituir a antiga simulação do Webots (`perse_webots`), oferecendo desacoplamento completo de renderização (execução *headless* nativa ultrarrápida), simulação realista de dinâmicas de contato (suspensão prismática com mola e amortecedor, geometria Ackermann, atrito pneu-solo e bumper mecânico móvel), além de um **AI Harness** via **Model Context Protocol (MCP)** para interação bidirecional com agentes autônomos de IA.

---

## 🚀 Funcionalidades

- **Modelo Físico Completo do Robô Perse (`perse_robot.xml`):**
  - **Inércia e CoM Reais:** Chassi baseado na plataforma Traxxas Stampede ($3.5\,\text{kg}$) com matriz completa de inércia e centro de massa medido.
  - **Suspensão Independente nas 4 Rodas:** Juntas prismáticas lineares (`slide`) com rigidez $k = 1500\,\text{N/m}$ e amortecimento $c = 50\,\text{Ns/m}$, proporcionando assentamento suave e absorção de desníveis.
  - **Direção Ackermann Dianteira:** Juntas de esterçamento (`steer_fl_joint`, `steer_fr_joint`) com controle de posição via atuadores angulares ($\pm 0.8\,\text{rad} \approx \pm 45^\circ$).
  - **Tração Traseira com Rodas Reais:** Juntas de rotação contínua nos eixos traseiros acionadas por atuadores de velocidade com atrito de borracha elástica.
  - **Bumper Frontal Móvel:** Lâmina parachoque móvel montada sobre junta prismática linear com curso de $3\,\text{cm}$, mola de retorno de $2500\,\text{N/m}$ e batente rígido para detecção e absorção de colisões.
  - **Sensores Embarcados:** Sites calibrados para IMU (acelerômetro + giroscópio), LiDAR 2D (`RpLidarA2`), Câmera frontal e Sensor Indutivo.
  - **Malhas 3D Alinhadas (STLs):** Chassi mecânico (`locomocao`), tampa superior (`tampa`), rodas Traxxas, bumper, LiDAR e câmera.
- **Simulação Headless Determinística:** O loop de física roda em processo ROS 2 desacoplado de interface gráfica, com publicação de `/clock` e taxa de 500 Hz com integrador implícito `implicitfast`.
- **Ponte IPC de Alta Performance:** Comunicação via Unix Domain Socket (`/tmp/perse_mujoco.sock`) permitindo controle de tempo e coleta de telemetria sem interferir na estabilidade do motor de física.
- **Controle Temporal Híbrido:**
  - `Free Run`: Execução contínua sincronizada com o relógio ROS 2.
  - `Step Mode`: Execução passo-a-passo sob demanda (`sim_step(N)`), ideal para testes automatizados, depuração e IA.
- **AI Harness (MCP):** Servidor MCP integrado (`perse_mujoco_mcp`) baseado no SDK oficial da Anthropic (`MCPServer`), expondo ferramentas de inspeção, telemetria, controle de câmera e captura de screenshots.
- **Suite de Testes Automatizados:** Testes unitários com validação estática e dinâmica (`test/test_perse_model.py`).

---

## 📁 Estrutura do Pacote

```
perse_mujoco/
├── CMakeLists.txt                  # Configuração CMake com fallback inteligente (find_package / FetchContent)
├── package.xml                     # Metadados e dependências ROS 2 Humble
├── README.md                       # Documentação principal do pacote
├── SPEC.md                         # Especificação técnica completa da simulação
├── AI_HARNESS_SPEC.md              # Especificação de arquitetura do harness de IA
├── config/                         # Parâmetros de simulação e arenas
├── include/perse_mujoco/           # Headers C++ (apenas declarações)
│   ├── MujocoIpcBridge.hpp         # Interface da ponte IPC via Unix Domain Socket
│   ├── MujocoViewer.hpp            # Interface do visualizador nativo GLFW
│   └── TestSimNode.hpp             # Declaração do nó de simulação ROS 2
├── src/                            # Implementações C++
│   ├── MujocoIpcBridge.cpp         # Implementação do servidor socket e comandos JSON
│   ├── MujocoViewer.cpp            # Visualizador nativo GLFW com suporte a screenshot e IPC
│   ├── TestSimNode.cpp             # Lógica de ciclo de vida, step e publicação de clock
│   ├── perse_mujoco_node.cpp       # Entrypoint do nó de simulação
│   └── perse_mujoco_viewer.cpp     # Entrypoint do nó visualizador
├── model/                          # Modelos MJCF e malhas 3D
│   ├── perse_robot.xml             # Modelo MJCF oficial e autocontido do robô Perse
│   ├── test_scene.xml              # Cenário de teste com robô Perse, arena, solo e obstáculos
│   └── meshes/                     # Malhas 3D processadas (OBJs com normais suaves e STLs)
│       ├── locomocao_crease.obj    # Chassi mecânico com normais de vértice suaves
│       ├── tampa_crease.obj        # Tampa/carenagem superior com crease-shading
│       ├── bumper_base_fixed.obj   # Suporte de montagem fixo aparafusado no chassi
│       ├── bumper_moving.obj       # Botão móvel do parachoque (aletas telescópicas + barra)
│       ├── wheel_aligned.stl       # Roda alinhada no eixo Y
│       ├── rplidar_a2.stl          # Malha oficial do Slamtec RpLidar A2 (Webots)
│       └── camera_centered.stl     # Malha da câmera frontal
├── launch/                         # Launch files em formato XML
│   ├── sim_test.launch             # Inicialização do nó de simulação física
│   └── viewer.launch               # Inicialização da janela interativa GLFW
├── test/                           # Testes automatizados
│   └── test_perse_model.py         # Testes de assentamento, direção, tração e bumper
└── python/perse_mujoco/            # Módulos Python
    ├── test_renderer.py            # Renderizador offscreen para testes visuais
    └── mcp_server/                 # Servidor e cliente MCP
        ├── ipc_client.py           # Cliente IPC Unix Domain Socket
        └── server.py               # Servidor MCP stdio (ferramentas de IA)
```

---

## 🛠️ Compilação e Execução

Dentro do container de desenvolvimento (`perse_container`):

```bash
# Compilar o pacote
colcon build --symlink-install --packages-select perse_mujoco

# Carregar o workspace
source install/setup.bash

# Rodar a simulação headless (com nó ROS 2 e IPC)
ros2 launch perse_mujoco sim_test.launch

# Rodar o visualizador nativo independente (abre janela GLFW MuJoCo)
ros2 launch perse_mujoco viewer.launch
```

> **Nota:** O visualizador `perse_mujoco_viewer` é desacoplado do nó de simulação. Você pode fechar e reabrir a simulação (`sim_test.launch`) à vontade, e a janela do visualizador permanecerá aberta, reconectando-se automaticamente assim que a simulação reiniciar!

### 🧪 Execução da Suite de Testes

Para validar a integridade física do robô (assentamento no solo, suspensão, ângulos de esterçamento, aceleração longitudinal e recuo do bumper sob colisão):

```bash
python3 src/perse_mujoco/test/test_perse_model.py
```

### 🖱️ Controles e Gestos Interativos na Janela (MuJoCo Studio Standard):
| Ação | Comando do Mouse / Teclado |
|---|---|
| **Rotacionar Câmera** | Clique e arraste com o **Botão Esquerdo** |
| **Transladar Câmera (Pan)** | Clique e arraste com o **Botão Direito** |
| **Zoom** | **Scroll** do mouse ou arraste com **Botão do Meio** |
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
