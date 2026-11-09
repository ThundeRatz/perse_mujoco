cmake_minimum_required(VERSION 3.11)
project(perse_mujoco)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(perse_lib REQUIRED)

# ------------------------------------------------------------------------------
# 1. Tenta encontrar o MuJoCo instalado no sistema via find_package
# ------------------------------------------------------------------------------
find_package(mujoco QUIET)

# 2. Se nao encontrou via find_package oficial, tenta via heuristica de caminhos
if(NOT mujoco_FOUND)
  # Procura no ambiente python do sistema ou de usuario se disponivel
  execute_process(
    COMMAND python3 -c "import os, mujoco; print(os.path.dirname(mujoco.__file__))"
    OUTPUT_VARIABLE PYTHON_MUJOCO_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )

  find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    PATHS
      ${PYTHON_MUJOCO_DIR}/include
      /usr/local/include
      /usr/include
      /opt/mujoco/include
      "$ENV{HOME}/.local/lib/python3.10/site-packages/mujoco/include"
  )

  find_library(MUJOCO_LIBRARY
    NAMES mujoco libmujoco.so.3.17.0 libmujoco.so
    PATHS
      ${PYTHON_MUJOCO_DIR}
      /usr/local/lib
      /usr/lib
      /opt/mujoco/lib
      "$ENV{HOME}/.local/lib/python3.10/site-packages/mujoco"
  )

  if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
    set(MUJOCO_FOUND TRUE)
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
      IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    )
    message(STATUS "MuJoCo encontrado no sistema: ${MUJOCO_LIBRARY}")
  endif()
endif()

# ------------------------------------------------------------------------------
# 3. Se ainda assim nao encontrou, usa FetchContent para baixar a release oficial
# ------------------------------------------------------------------------------
if(NOT mujoco_FOUND AND NOT MUJOCO_FOUND)
  message(STATUS "MuJoCo nao encontrado no sistema. Baixando via FetchContent...")
  include(FetchContent)
  FetchContent_Declare(
    mujoco_binary
    URL https://github.com/google-deepmind/mujoco/releases/download/3.2.0/mujoco-3.2.0-linux-x86_64.tar.gz
  )
  FetchContent_MakeAvailable(mujoco_binary)

  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${mujoco_binary_SOURCE_DIR}/include"
    IMPORTED_LOCATION "${mujoco_binary_SOURCE_DIR}/lib/libmujoco.so.3.2.0"
  )
  set(MUJOCO_LIBRARY "${mujoco_binary_SOURCE_DIR}/lib/libmujoco.so.3.2.0")
  message(STATUS "MuJoCo configurado via FetchContent: ${MUJOCO_LIBRARY}")
endif()

find_package(glfw3 REQUIRED)

# ------------------------------------------------------------------------------
# Executavel do no de simulacao C++
# ------------------------------------------------------------------------------
add_executable(perse_mujoco_node
  src/perse_mujoco_node.cpp
  src/lib/PerseMujocoNode.cpp
  src/lib/ActuatorBridge.cpp
  src/lib/SensorBridge.cpp
  src/lib/SyntheticVision.cpp
  src/lib/MujocoIpcBridge.cpp
)

target_include_directories(perse_mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(perse_mujoco_node
  mujoco::mujoco
  yaml-cpp
)

if(TARGET mujoco::mujoco)
  get_target_property(MUJOCO_LIB_LOC mujoco::mujoco IMPORTED_LOCATION)
  if(MUJOCO_LIB_LOC)
    get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB_LOC}" DIRECTORY)
    set_target_properties(perse_mujoco_node PROPERTIES
      BUILD_RPATH "${MUJOCO_LIB_DIR}"
      INSTALL_RPATH "${MUJOCO_LIB_DIR}"
    )
  endif()
endif()

ament_target_dependencies(perse_mujoco_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  vision_msgs
  rosgraph_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ------------------------------------------------------------------------------
# Executavel do visualizador nativo independente C++
# ------------------------------------------------------------------------------
add_executable(perse_mujoco_viewer
  src/perse_mujoco_viewer.cpp
  src/lib/MujocoViewer.cpp
)

target_include_directories(perse_mujoco_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(perse_mujoco_viewer
  mujoco::mujoco
  glfw
)

if(TARGET mujoco::mujoco)
  get_target_property(MUJOCO_LIB_LOC mujoco::mujoco IMPORTED_LOCATION)
  if(MUJOCO_LIB_LOC)
    get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB_LOC}" DIRECTORY)
    set_target_properties(perse_mujoco_viewer PROPERTIES
      BUILD_RPATH "${MUJOCO_LIB_DIR}"
      INSTALL_RPATH "${MUJOCO_LIB_DIR}"
    )
  endif()
endif()

ament_target_dependencies(perse_mujoco_viewer
  ament_index_cpp
)

# ------------------------------------------------------------------------------
# Executavel do visualizador Rerun C++
# ------------------------------------------------------------------------------
add_executable(perse_mujoco_rerun
  src/perse_mujoco_rerun.cpp
  src/lib/MujocoRerun.cpp
)

target_include_directories(perse_mujoco_rerun PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(perse_mujoco_rerun
  mujoco::mujoco
  perse_lib::perse_lib
  rerun_sdk
)

if(TARGET mujoco::mujoco)
  get_target_property(MUJOCO_LIB_LOC mujoco::mujoco IMPORTED_LOCATION)
  if(MUJOCO_LIB_LOC)
    get_filename_component(MUJOCO_LIB_DIR "${MUJOCO_LIB_LOC}" DIRECTORY)
    set_target_properties(perse_mujoco_rerun PROPERTIES
      BUILD_RPATH "${MUJOCO_LIB_DIR}"
      INSTALL_RPATH "${MUJOCO_LIB_DIR}"
    )
  endif()
endif()

ament_target_dependencies(perse_mujoco_rerun
  rclcpp
  ament_index_cpp
  perse_lib
)

# ------------------------------------------------------------------------------
# Geracao automatica do perse_robot.xml a partir de perse_robot.xml.xacro e perse_properties.yaml
# ------------------------------------------------------------------------------
find_package(perse_bringup QUIET)

set(XACRO_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/model/perse_robot.xml.xacro")
set(PERSE_PROPS "${CMAKE_CURRENT_SOURCE_DIR}/../perse_bringup/config/perse_properties.yaml")
if(perse_bringup_FOUND)
  set(PERSE_PROPS "${perse_bringup_DIR}/../../../share/perse_bringup/config/perse_properties.yaml")
endif()
set(ROBOT_XML_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/model/perse_robot.xml")

add_custom_target(generate_perse_robot_xml ALL
  COMMAND /opt/ros/humble/bin/xacro "${XACRO_INPUT}" "properties_file:=${PERSE_PROPS}" > "${ROBOT_XML_OUTPUT}"
  BYPRODUCTS "${ROBOT_XML_OUTPUT}"
  COMMENT "Gerando perse_robot.xml a partir do xacro e perse_properties.yaml..."
)

add_dependencies(perse_mujoco_node generate_perse_robot_xml)
add_dependencies(perse_mujoco_viewer generate_perse_robot_xml)
add_dependencies(perse_mujoco_rerun generate_perse_robot_xml)

# Instalacao de binarios e recursos
install(TARGETS
  perse_mujoco_node
  perse_mujoco_viewer
  perse_mujoco_rerun
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  python/perse_mujoco/map_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY model config launch python test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
