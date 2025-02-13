cmake_minimum_required(VERSION 3.0.2)
project(tum_helpers_cpp)

## Compiler Option
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Package Dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(tum_types_cpp REQUIRED)

## catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Eigen3 tum_types_cpp
)

include_directories(${EIGEN3_INCLUDE_DIRS})

## HEADER-ONLY LIBRARY TARGET
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
)

target_compile_options(${PROJECT_NAME} INTERFACE -Ofast)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

target_link_libraries(${PROJECT_NAME} INTERFACE tum_types_cpp)


install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES export_${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
