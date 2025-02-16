cmake_minimum_required(VERSION 2.8.3)
project(ssa_estimation_node_cpp)

set(NODE_EXECUTABLE_NAME ssa_estimation_node)
set(NODE_EXECUTABLE_PATH src/ssa_estimation_node_main.cpp)

set(NODE_DEPENDENCIES
 ros_debug_helpers_cpp
 tum_types_cpp
 tum_helpers_cpp
 msgs
 nav_msgs
 sensor_msgs
 ssa_estimation_constants
 ssa_estimation_cpp
 diagnostic_msgs
 tum_type_conversions_ros_cpp
 ros1_watchdog_cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
 roscpp
 ${NODE_DEPENDENCIES}
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp ${NODE_DEPENDENCIES}
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})
target_compile_options(${NODE_EXECUTABLE_NAME} PUBLIC -Ofast)
target_link_libraries(${NODE_EXECUTABLE_NAME}
 ${catkin_LIBRARIES}
)

install(TARGETS ${NODE_EXECUTABLE_NAME}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
