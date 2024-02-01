cmake_minimum_required(VERSION 3.8)
project(zlac8015d_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_executable(example src/example.cpp src/zlac8015d.cpp)
ament_target_dependencies(example serial rclcpp)

add_executable(zlac_run src/zlac_run.cpp src/zlac8015d.cpp)
ament_target_dependencies(zlac_run
  serial
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

add_executable(zlac_run_dummy src/zlac_run_dummy.cpp src/zlac8015d.cpp)
ament_target_dependencies(zlac_run_dummy
  serial
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
)

add_library(zlac8015d src/zlac8015d.cpp) 
ament_export_targets(zlac8015d HAS_LIBRARY_TARGET)
ament_target_dependencies(zlac8015d serial rclcpp)

install(
  DIRECTORY include
  DESTINATION include
)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS 
  example
  zlac_run
  zlac_run_dummy
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS zlac8015d
  EXPORT zlac8015d
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
