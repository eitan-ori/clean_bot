cmake_minimum_required(VERSION 3.8)
project(my_robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/arduino_bridge.py
  scripts/imu_driver.py
  scripts/coverage_planner.py
  scripts/simple_simulation.py
  scripts/dock_detector.py
  scripts/auto_docker.py
  scripts/cmd_vel_mux.py
  scripts/demo_mission.py
  scripts/complex_mission.py
  scripts/autonomous_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(test/test_launch.py)
  add_launch_test(test/test_simulation_integration.py)
  ament_add_pytest_test(test_docking_logic test/test_docking_logic.py)
  ament_add_pytest_test(test_explorer_logic test/test_explorer_logic.py)
endif()

ament_package()
