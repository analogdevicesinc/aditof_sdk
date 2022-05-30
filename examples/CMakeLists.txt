cmake_minimum_required(VERSION 2.8)
project(examples)

############################### Options #######################################
option(WITH_IMGUI "Build ImGui based examples?" OFF)
find_package(OpenCV)

# Dependecies that can be accessed by any example
if (WITH_IMGUI)
    add_subdirectory(deps/imgui)
endif()

# Examples
if (OPENCV_FOUND)
    add_subdirectory(aditof-demo)
endif()
if (WITH_IMGUI)
    add_subdirectory(aditof-demo-imgui)
endif()
add_subdirectory(first-frame)

if (WITH_NETWORK)
    add_subdirectory(first-frame-network)
endif()

if (XAVIER)
    add_subdirectory(imshow-xavier)
endif()

include(GNUInstallDirs)

install(PROGRAMS
    $<TARGET_FILE:first-frame>
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if (OPENCV_FOUND)
    install(PROGRAMS
        $<TARGET_FILE:aditof-demo>
        DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

if(WIN32)
    include(FindOpenSSL)
    if(OPENCV_FOUND)
    add_custom_target(copy-dll-example
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-demo>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:first-frame>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:first-frame-network>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-demo>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:first-frame>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:first-frame-network>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    else()
    add_custom_target(copy-dll-example
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:first-frame>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:first-frame-network>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:first-frame>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:first-frame-network>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    
endif()
