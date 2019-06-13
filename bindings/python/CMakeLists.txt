cmake_minimum_required(VERSION 2.8)

find_package(pybind11 REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(../../sdk/includes)

pybind11_add_module(aditofpython aditofpython.cpp)

target_link_libraries(aditofpython PRIVATE aditof)
