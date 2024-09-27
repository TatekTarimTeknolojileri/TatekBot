cmake_minimum_required(VERSION 3.5)
project(tatekbot_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

set(dependencies
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

install(DIRECTORY  launch meshes urdf worlds textures include src rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  launch/robot_sim.launch.py
  launch/robot_state_publisher.launch.py
  launch/tatekbot_display.launch.py
 
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()

