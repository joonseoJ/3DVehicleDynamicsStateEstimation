cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME tum_type_conversions_ros_cpp)
project(${PROJECT_NAME})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  tum_types_cpp
  tum_helpers_cpp
  geometry_msgs
  msgs
  sensor_msgs
  diagnostic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf nav_msgs tum_types_cpp tum_helpers_cpp geometry_msgs msgs sensor_msgs diagnostic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/tum_type_conversions.cpp
  src/orientation.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
  geometry_msgs_generate_messages_cpp
  msgs_generate_messages_cpp
  sensor_msgs_generate_messages_cpp
  diagnostic_msgs_generate_messages_cpp
  nav_msgs_generate_messages_cpp
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(orientation_conversion_tests test/orientation_tests.test test/orientation_tests.cpp)
#   target_link_libraries(orientation_conversion_tests ${PROJECT_NAME} ${catkin_LIBRARIES})
# endif()
