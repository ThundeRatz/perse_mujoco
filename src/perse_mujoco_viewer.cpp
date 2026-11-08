#include "perse_mujoco/MujocoViewer.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  std::string model_path = "";
  if (argc > 1 && std::string(argv[1]).size() > 0) {
    model_path = argv[1];
  }

  if (model_path.empty()) {
    try {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("perse_mujoco");
      std::string gen_path = pkg_share + "/model/generated_scene.xml";
      // Se houver cena gerada recentemente, usa ela; caso contrário usa
      // default_scene.xml
      FILE *f = fopen(gen_path.c_str(), "r");
      if (f) {
        fclose(f);
        model_path = gen_path;
      } else {
        model_path = pkg_share + "/model/default_scene.xml";
      }
    } catch (...) {
      model_path = "model/default_scene.xml";
    }
  }

  std::cout
      << "[perse_mujoco_viewer] Iniciando visualizador nativo independente..."
      << std::endl;
  std::cout << "[perse_mujoco_viewer] Carregando modelo: " << model_path
            << std::endl;

  perse_mujoco::MujocoViewer viewer(model_path);
  if (!viewer.init()) {
    std::cerr << "[perse_mujoco_viewer] Falha ao inicializar o visualizador."
              << std::endl;
    return 1;
  }

  std::cout << "[perse_mujoco_viewer] Visualizador ativo! Pronto para "
               "sincronizar com simulação."
            << std::endl;
  viewer.run();
  viewer.stop();

  std::cout << "[perse_mujoco_viewer] Visualizador finalizado." << std::endl;
  return 0;
}
