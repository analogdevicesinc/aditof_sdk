cmake_minimum_required(VERSION 2.8)
project(uvc-gadget)

add_executable(${PROJECT_NAME} uvc-gadget.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config_pipe.sh
               ${CMAKE_CURRENT_BINARY_DIR}/config_pipe.sh
               COPYONLY)
