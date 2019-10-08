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

if(WIN32)
    include(FindOpenSSL)
    add_custom_target(copy-dll-opencv
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-imshow>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-dnn>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-imshow>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-dnn>
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
