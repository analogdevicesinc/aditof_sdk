cmake_minimum_required(VERSION 2.8)

project(aditof-opencv-imshow)

add_executable(${PROJECT_NAME} main.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${OpenCV_LIBS})
