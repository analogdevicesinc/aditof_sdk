cmake_minimum_required(VERSION 2.8)
project(eeprom-tool)

if (MSVC)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../deps/wingetopt/src)
    set(GETOPT_C_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../deps/wingetopt/src/getopt.c)
endif (MSVC)

add_executable(${PROJECT_NAME} main.cpp eeprom_tool.cpp cam96tof1_eeprom.cpp cli_helper.cpp camera_eeprom_factory.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} PRIVATE aditof)
