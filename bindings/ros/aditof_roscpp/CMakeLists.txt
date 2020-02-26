cmake_minimum_required(VERSION 2.8.3)
project(aditof_roscpp)

option(WITH_EXAMPLES "Build examples?" ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(aditof REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

if (WITH_EXAMPLES)
    find_package(catkin REQUIRED COMPONENTS
      rviz
    )

    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

    add_executable(aditof_rviz_pcl ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/rviz_pointcloud.cpp ${SOURCES})
    target_sources(aditof_rviz_pcl INTERFACE ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
    target_link_libraries(aditof_rviz_pcl ${aditof_LIBRARIES})
    target_link_libraries(aditof_rviz_pcl ${catkin_LIBRARIES})
endif()
