cmake_minimum_required(VERSION 3.5)
project(aditof_roscpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(WITH_ROS_EXAMPLES "Build ROS examples" OFF)

list(APPEND CMAKE_PREFIX_PATH ${ADITOF_CMAKE_INSTALL_PREFIX}
     ${ADITOF_CMAKE_PREFIX_PATH})

     #find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(aditof ${VERSION} REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Aditof_roscpp.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure
  DEPENDS aditof)

# include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS}
#                     ${aditof_INCLUDE_DIRS})

include_directories(include)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_node.cpp)

add_executable(aditof_camera_node
               ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_node.cpp ${SOURCES})
target_sources(aditof_camera_node
               INTERFACE ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
target_link_libraries(aditof_camera_node aditof::aditof)
target_link_libraries(aditof_camera_node ${catkin_LIBRARIES})
add_dependencies(aditof_camera_node ${PROJECT_NAME}_gencfg)

if(WITH_ROS_EXAMPLES)
  find_package(catkin REQUIRED COMPONENTS rviz)

  add_executable(
    aditof_rviz_pcl ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/rviz_pointcloud.cpp
                    ${SOURCES})
  target_sources(aditof_rviz_pcl
                 INTERFACE ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
  target_link_libraries(aditof_rviz_pcl aditof::aditof)
  target_link_libraries(aditof_rviz_pcl ${catkin_LIBRARIES})

endif()
