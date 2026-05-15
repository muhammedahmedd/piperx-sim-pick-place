cmake_minimum_required(VERSION 3.8)
project(piperx_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(piperx_sim_control src/piperx_sim_control.cpp)

ament_target_dependencies(piperx_sim_control
  rclcpp
  sensor_msgs
)

install(TARGETS
  piperx_sim_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
