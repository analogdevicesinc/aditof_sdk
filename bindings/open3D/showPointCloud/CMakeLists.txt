cmake_minimum_required(VERSION 2.8)

project(aditof-open3d-showPointCloud)

message(STATUS "Open3D ${Open3D_VERSION}")

# link_directories must be before add_executable
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} main.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${Open3D_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})
