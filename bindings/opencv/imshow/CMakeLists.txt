cmake_minimum_required(VERSION 2.8)

project(aditof-opencv-imshow)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${OpenCV_LIBS})
