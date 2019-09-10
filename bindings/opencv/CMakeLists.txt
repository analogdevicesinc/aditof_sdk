cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

add_subdirectory(imshow)
add_subdirectory(dnn)

# Install the helper header for opencv in include/aditof
include(GNUInstallDirs)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/aditof_opencv.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aditof
)
