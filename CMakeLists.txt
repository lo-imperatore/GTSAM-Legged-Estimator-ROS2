cmake_minimum_required(VERSION 3.16)
project(gtsam_legged_replay_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GTSAM_LEGGED_USE_GALILEAN_IMU
  "Use Galilean preintegration and IMU factors in the fixed-lag smoothers" ON)

find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(Eigen3 REQUIRED)
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
      "Python executable used by ament_cmake" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
      "Python executable used by legacy CMake dependencies" FORCE)
endif()
find_package(ament_cmake REQUIRED)
find_package(any_msgs)
find_package(anymal_msgs)
find_package(unitree_hg)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(legged_estimator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pinocchio)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_library(gtsam_legged_backend STATIC
  src/gtsam/navigation/LeggedEstimator.cpp
)
target_include_directories(gtsam_legged_backend
  BEFORE PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(gtsam_legged_backend
  PUBLIC
    gtsam
    gtsam_unstable
)
if(GTSAM_LEGGED_USE_GALILEAN_IMU)
  target_compile_definitions(gtsam_legged_backend
    PUBLIC GTSAM_LEGGED_USE_GALILEAN_IMU=1)
  message(STATUS "Legged smoother IMU backend: Galilean")
else()
  message(STATUS "Legged smoother IMU backend: conventional")
endif()

add_executable(LeggedEstimatorRos2Node
  src/JointTorqueGrfEstimator.cpp
  src/LeggedEstimatorRos2Node.cpp
  src/SpotContactAdapter.cpp
)

if(any_msgs_FOUND AND anymal_msgs_FOUND AND pinocchio_FOUND)
  target_sources(LeggedEstimatorRos2Node PRIVATE src/AnymalContactAdapter.cpp)
  target_compile_definitions(LeggedEstimatorRos2Node PRIVATE
    GTSAM_LEGGED_HAVE_ANYMAL_MSGS=1)
else()
  message(WARNING
    "any_msgs/anymal_msgs/pinocchio were not found; robot.type=anymal support will be disabled")
endif()

if(pinocchio_FOUND)
  target_compile_definitions(LeggedEstimatorRos2Node PRIVATE
    GTSAM_LEGGED_HAVE_PINOCCHIO=1)
endif()

if(unitree_hg_FOUND)
  target_sources(LeggedEstimatorRos2Node PRIVATE src/UnitreeG1Adapter.cpp)
  target_compile_definitions(LeggedEstimatorRos2Node PRIVATE
    GTSAM_LEGGED_HAVE_UNITREE_HG=1)
endif()

target_include_directories(LeggedEstimatorRos2Node
  BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(LeggedEstimatorRos2Node
  gtsam_legged_backend
  gtsam
  gtsam_unstable
)
ament_target_dependencies(LeggedEstimatorRos2Node
  builtin_interfaces
  geometry_msgs
  legged_estimator_msgs
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  rosidl_typesupport_introspection_cpp
)
if(any_msgs_FOUND AND anymal_msgs_FOUND AND pinocchio_FOUND)
  ament_target_dependencies(LeggedEstimatorRos2Node any_msgs anymal_msgs)
endif()
if(pinocchio_FOUND)
  target_link_libraries(LeggedEstimatorRos2Node pinocchio::pinocchio)
endif()
if(unitree_hg_FOUND)
  ament_target_dependencies(LeggedEstimatorRos2Node unitree_hg)
else()
  message(WARNING
    "unitree_hg was not found; robot.type=g1 support will be disabled")
endif()

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
