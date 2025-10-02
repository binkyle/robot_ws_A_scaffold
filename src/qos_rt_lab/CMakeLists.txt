cmake_minimum_required(VERSION 3.5)
project(qos_rt_lab)

# 默认禁用测试，减少依赖
option(BUILD_TESTING "Build tests" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(qos_rt_pub src/qos_rt_pub.cpp)
ament_target_dependencies(qos_rt_pub rclcpp std_msgs std_srvs)

add_executable(qos_rt_sub src/qos_rt_sub.cpp)
ament_target_dependencies(qos_rt_sub rclcpp std_msgs)

install(TARGETS
  qos_rt_pub
  qos_rt_sub
  DESTINATION lib/${PROJECT_NAME})

# 可选的 launch 安装（有就装，没有也不报错）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
