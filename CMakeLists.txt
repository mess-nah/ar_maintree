cmake_minimum_required(VERSION 3.8)
project(ar_maintree)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs)

add_executable(forest_nav_node
  src/servers/forestnavServer.cpp
  include/robotlib/controllers/pid.cpp 
)
add_executable(plan_path_client src/plan_path_client.cpp)

ament_target_dependencies(
  ar_maintree
  forest_nav_node
  plan_path_client
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS forest_nav_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS plan_path_client
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
