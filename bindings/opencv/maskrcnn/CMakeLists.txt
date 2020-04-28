cmake_minimum_required(VERSION 2.8)

project(aditof-opencv-maskrcnn)

add_executable(${PROJECT_NAME} main.cpp)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mscoco_labels.names
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${OpenCV_LIBS})
