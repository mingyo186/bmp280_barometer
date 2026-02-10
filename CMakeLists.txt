cmake_minimum_required(VERSION 3.8)
project(bmp280_barometer)

# Default to C++17 / C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ──────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Install Python package (bmp280_barometer/) ─────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install executable node script ────────────────────────────
install(PROGRAMS
  nodes/bmp280_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch files ──────────────────────────────────────
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install config files ──────────────────────────────────────
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Linters (optional) ────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
