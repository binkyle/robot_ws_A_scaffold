
cmake_minimum_required(VERSION 3.8)
project(qos_rt_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(qos_rt_pub src/qos_rt_pub.cpp)
ament_target_dependencies(qos_rt_pub rclcpp std_msgs std_srvs)

add_executable(qos_rt_sub src/qos_rt_sub.cpp)
ament_target_dependencies(qos_rt_sub rclcpp std_msgs diagnostic_updater)

add_executable(qos_supervisor src/qos_supervisor.cpp)
ament_target_dependencies(qos_supervisor rclcpp std_msgs std_srvs)

install(TARGETS qos_rt_pub qos_rt_sub qos_supervisor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
