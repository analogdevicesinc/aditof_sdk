cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

add_subdirectory(imshow)

# Install the helper header for opencv in include/aditof
include(GNUInstallDirs)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/aditof_opencv.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aditof
)

if(OPENCV_FOUND OR
   OPENCV_CORE_FOUND) # we check for this because the OpenCVConfig.cmake(v2.4.9.1) used on dragonboards fails to set OPENCV_FOUND
    if(OpenCV_VERSION VERSION_LESS "3.4.1")
        message(WARNING "The dnn example is only available for OpenCV version >= 3.4.1! This example won't be built!")
        add_definitions(-DOPENCV2)

        if(WIN32)
            include(FindOpenSSL)
            add_custom_target(copy-dll-opencv
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif()
    else()
        message(STATUS "The dnn example will be built (OpenCV version >= 3.4.1)")
        add_subdirectory(dnn)

        if(WIN32)
            include(FindOpenSSL)
            add_custom_target(copy-dll-opencv
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-dnn>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-dnn>
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif()
    endif()
endif()
