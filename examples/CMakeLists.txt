cmake_minimum_required(VERSION 2.8)
project(examples)

add_subdirectory(aditof-demo)
add_subdirectory(first-frame)
add_subdirectory(frame-filters)

include(GNUInstallDirs)

install(PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-demo/aditof-demo
    ${CMAKE_CURRENT_BINARY_DIR}/first-frame/first-frame
    ${CMAKE_CURRENT_BINARY_DIR}/frame-filters/frame-filters
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
