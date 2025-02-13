cmake_minimum_required(VERSION 2.8.3)
project(ssa_estimation_cpp)

set(DEPENDENCIES
 roscpp
 tum_types_cpp
 tum_helpers_cpp
 tum_type_conversions_ros_cpp
 ssa_estimation_constants
 ssa_estimation_base
 imu_handler
 classical_filter
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
 ${DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${DEPENDENCIES}
 DEPENDS EIGEN3
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(${PROJECT_NAME} INTERFACE -Ofast)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
 catkin_add_gtest(unit_test_runtime test/unit_test_runtime.cpp)
 target_link_libraries(unit_test_runtime ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()