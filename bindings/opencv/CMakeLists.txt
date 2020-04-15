cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)

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

        if(OpenCV_VERSION VERSION_LESS "3.0.0")
            message(STATUS "USING OPENCV Version 2")
            add_definitions(-DOPENCV2)
        endif(OpenCV_VERSION VERSION_LESS "3.0.0")

        if(WIN32)
            include(FindOpenSSL)
            add_custom_target(copy-dll-opencv
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-opencv-imshow>
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        endif()
    else()
        add_subdirectory(dnn)
        if(OpenCV_VERSION VERSION_LESS "3.4.3")
            message(WARNING "The maskR-CNN example is only available for OpenCV version >= 3.4.3! This example won't be built!")
        else()
            add_subdirectory(maskrcnn)
        endif()
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

add_subdirectory(imshow)
