cmake_minimum_required(VERSION 2.8)
project(eeprom-tool LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)

add_executable(${PROJECT_NAME} main.cpp eeprom_tool.cpp cam96tof1_eeprom.cpp cli_helper.cpp camera_eeprom_factory.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} PRIVATE aditof)
if (MSVC)
    target_link_libraries(${PROJECT_NAME} PRIVATE wingetopt)
endif (MSVC)
