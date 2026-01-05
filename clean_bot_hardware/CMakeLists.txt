cmake_minimum_required(VERSION 3.8)
project(clean_bot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  clean_bot_hardware/rplidar_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rplidar_test
)

install(PROGRAMS
  clean_bot_hardware/imu_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_publisher
)

install(PROGRAMS
  clean_bot_hardware/imu_odom_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_odom_broadcaster
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
