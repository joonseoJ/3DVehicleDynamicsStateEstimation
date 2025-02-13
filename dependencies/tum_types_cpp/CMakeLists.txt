cmake_minimum_required(VERSION 3.0.2)
project(tum_types_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

catkin_package(
  INCLUDE_DIRS include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Uncomment to add tests if needed
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(unit_tests
#     test/unit_test.cpp
#   )
#   target_link_libraries(unit_tests ${PROJECT_NAME})
# endif()
