cmake_minimum_required(VERSION 3.0.2)
project(state_estimation_node_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 찾을 패키지들 정의
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_debug_helpers_cpp
  tum_types_cpp
  tum_helpers_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  msgs
  nav_msgs
  geometry_msgs
  state_estimation_cpp
  state_estimation_base
  state_estimation_constants
  sensor_msgs
  diagnostic_msgs
  tum_type_conversions_ros_cpp
  ros1_watchdog_cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    ros_debug_helpers_cpp
    tum_types_cpp
    tum_helpers_cpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    msgs
    nav_msgs
    geometry_msgs
    state_estimation_cpp
    state_estimation_base
    state_estimation_constants
    sensor_msgs
    diagnostic_msgs
    tum_type_conversions_ros_cpp
    ros1_watchdog_cpp
)

# Include 디렉토리 설정
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 노드 실행 파일 이름 및 소스 경로 설정
set(NODE_EXECUTABLE_NAMES 
  state_estimation_node_cpp_ekf_2d_kin
  state_estimation_node_cpp_ekf_3d_kin
  state_estimation_node_cpp_ekf_2d_nh
  state_estimation_node_cpp_ekf_3d_nh
  state_estimation_node_cpp_ekf_3d_stm
)

set(NODE_EXECUTABLE_PATHS 
  src/cpp/kinematic/state_estimation_node_cpp_ekf_2d_main.cpp
  src/cpp/kinematic/state_estimation_node_cpp_ekf_3d_main.cpp
  src/cpp/non_holonomic/state_estimation_node_cpp_ekf_2d_main.cpp
  src/cpp/non_holonomic/state_estimation_node_cpp_ekf_3d_main.cpp
  src/cpp/single_track_model/state_estimation_node_cpp_ekf_3d_main.cpp
)

list(LENGTH NODE_EXECUTABLE_NAMES COUNT)
math(EXPR COUNT "${COUNT} - 1")

# Executable 생성 루프
foreach(IDX RANGE ${COUNT})
  list(GET NODE_EXECUTABLE_NAMES ${IDX} NODE_EXECUTABLE_NAME)
  list(GET NODE_EXECUTABLE_PATHS ${IDX} NODE_EXECUTABLE_PATH)

  add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})
  
  add_dependencies(${NODE_EXECUTABLE_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(${NODE_EXECUTABLE_NAME}
    ${catkin_LIBRARIES}
  )

  target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -Ofast)

  install(TARGETS ${NODE_EXECUTABLE_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

# Include 파일 설치
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
  