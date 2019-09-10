cmake_minimum_required(VERSION 2.8)
project(examples)

add_subdirectory(aditof-demo)
add_subdirectory(first-frame)

include(GNUInstallDirs)

install(PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-demo/aditof-demo
    ${CMAKE_CURRENT_BINARY_DIR}/first-frame/first-frame
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
