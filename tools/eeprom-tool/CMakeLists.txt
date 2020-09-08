cmake_minimum_required(VERSION 2.8)
project(eeprom-tool)
find_package(aditof)

if (MSVC)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../deps/wingetopt/src)
    set(GETOPT_C_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../deps/wingetopt/src/getopt.c)
endif (MSVC)


add_executable(${PROJECT_NAME} main.cpp eeprom_tool.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${aditof_LIBRARIES})
