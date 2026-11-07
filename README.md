# perse_mujoco

Pacote de simulação física baseada no **MuJoCo** para o robô **Perse**. Desenvolvido para substituir a antiga simulação do Webots (`perse_webots`), oferecendo desacoplamento completo de renderização (execução *headless* nativa ultrarrápida), simulação realista de dinâmicas de contato (suspensão, derrapagem na aceleração, atrito), além de um **AI Harness** via **Model Context Protocol (MCP)** para interação bidirecional com agentes autônomos de IA.

---

## 🚀 Funcionalidades

- **Simulação Headless Determinística:** O loop de física roda em processo ROS 2 desacoplado de interface gráfica, com publicação de `/clock` e suporte a taxas de 500–1000 Hz.
- **Ponte IPC de Alta Performance:** Comunicação via Unix Domain Socket (`/tmp/perse_mujoco.sock`) permitindo controle de tempo e coleta de telemetria sem interferir na estabilidade do motor de física.
- **Controle Temporal Híbrido:**
  - `Free Run`: Execução contínua sincronizada com o relógio ROS 2.
  - `Step Mode`: Execução passo-a-passo sob demanda (`sim_step(N)`), ideal para testes automatizados, depuração e IA.
- **AI Harness (MCP):** Servidor MCP integrado (`perse_mujoco_mcp`) baseado no SDK oficial da Anthropic (`MCPServer`), expondo ferramentas de inspeção, telemetria e perturbação dinâmica.

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
│   └── TestSimNode.hpp             # Declaração do nó de simulação ROS 2
├── src/                            # Implementações C++
│   ├── MujocoIpcBridge.cpp         # Implementação do servidor socket e comandos JSON
│   ├── TestSimNode.cpp             # Lógica de ciclo de vida, step e publicação de clock
│   └── perse_mujoco_node.cpp       # Entrypoint (função main())
├── model/                          # Modelos MJCF e malhas 3D
│   └── test_scene.xml              # Cenário de teste físico com plano e obstáculos
├── launch/                         # Launch files em formato XML
│   └── sim_test.launch             # Inicialização do nó de simulação de teste
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

# Rodar a simulação com o launch file XML de teste
# Rodar a simulação headless (com nó ROS 2 e IPC)
ros2 launch perse_mujoco sim_test.launch

# Rodar o visualizador nativo independente (abre janela GLFW MuJoCo)
ros2 launch perse_mujoco viewer.launch
```

> **Nota:** O visualizador `perse_mujoco_viewer` é desacoplado do nó de simulação. Você pode fechar e reabrir a simulação (`sim_test.launch`) à vontade, e a janela do visualizador permanecerá aberta, reconectando-se automaticamente assim que a simulação reiniciar!

### 🖱️ Controles e Gestos Interativos na Janela (MuJoCo Studio Standard):
| Ação | Comando do Mouse / Teclado |
|---|---|
| **Rotacionar Câmera** | Clique e arraste com o **Botão Esquerdo** |
| **Transladar Câmera (Pan)** | Clique e arraste com o **Botão Direito** |
| **Zoom** | **Scroll** do mouse ou arraste com **Botão do Meio** |
| **Centralizar Ponto Focal de Órbita** | **Duplo clique** (com botão esquerdo ou direito) em qualquer ponto ou objeto |
| **Puxar / Aplicar Força no Objeto ("Corda")** | Segure **Ctrl** + clique e arraste com o **Botão Direito** sobre o objeto |

Para validar a renderização offscreen e gerar um frame de teste:
```bash
python3 src/perse_mujoco/python/perse_mujoco/test_renderer.py --output src/perse_mujoco/test_render.png
```

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

### 2. Iniciar a Simulação e o Visualizador
```bash
# Terminal 1: Iniciar o nó core de física
ros2 launch perse_mujoco sim_test.launch

# Terminal 2 (opcional): Iniciar a janela nativa do visualizador
ros2 launch perse_mujoco viewer.launch
```

### 3. Ferramentas Disponíveis para o Agente

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
| `viewer_focus_body` | Centraliza e foca a câmera automaticamente sobre qualquer corpo (`test_box`, `test_cone`, etc.). |
| `viewer_set_option` | Ativa ou desativa modos de depuração visual (`contacts`, `joints`, `com`, `inertia`, `wireframe`, `shadow`, `transparent`). |
| `viewer_capture_screenshot` | Captura um screenshot em alta fidelidade diretamente da viewport do visualizador em tempo real. |
