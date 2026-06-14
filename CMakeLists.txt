cmake_minimum_required(VERSION 3.16)
project(gtsam_legged_replay_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
      "Python executable used by ament_cmake" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(anymal_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_subdirectory(spot_msgs)

add_executable(LeggedEstimatorRos2Node
  src/AnymalContactAdapter.cpp
  src/LeggedEstimatorRos2Node.cpp
  src/SpotContactAdapter.cpp
)

target_include_directories(LeggedEstimatorRos2Node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(LeggedEstimatorRos2Node
  gtsam
  gtsam_unstable
)
ament_target_dependencies(LeggedEstimatorRos2Node
  anymal_msgs
  builtin_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  rosidl_typesupport_introspection_cpp
)

install(TARGETS
  LeggedEstimatorRos2Node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
