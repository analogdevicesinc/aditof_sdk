cmake_minimum_required(VERSION 2.8)
project(adi_tof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 2)
set(ADITOF_VERSION_MINOR 0)
set(ADITOF_VERSION_PATCH 0)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)
option(WITH_OPENCV "Build opencv bindings?" OFF)
option(WITH_MATLAB "Build matlab bindings?" OFF)
option(WITH_OPEN3D "Build open3d bindings?" OFF)
option(WITH_ROS "Build ros bindings?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(USE_CHICONY "Use Chicony 006 camera?" OFF)
option(USE_FXTOF1 "Use FXTOF1 camera?" OFF)
option(WITH_TOOLS "Build tools?" OFF)

if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

if (USE_CHICONY)
        add_definitions(-DCHICONY_006)
endif()

if (USE_FXTOF1)
        add_definitions(-DFXTOF1)
endif()

add_subdirectory(sdk)
add_subdirectory(apps)

if(WITH_TOOLS)
        if (MSVC)
                add_subdirectory(tools/deps/wingetopt)
        endif(MSVC)
        add_subdirectory(tools/eeprom-tool)
endif()
if (WITH_EXAMPLES)
        add_subdirectory(examples)
endif()
if (WITH_DOC)
        add_subdirectory(doc)
endif()
if (WITH_PYTHON)
        add_subdirectory(bindings/python)
endif()
if (WITH_OPENCV)
        add_subdirectory(bindings/opencv)
endif()
if (WITH_MATLAB)
        add_subdirectory(bindings/matlab)
endif()
if (WITH_OPEN3D)
        add_subdirectory(bindings/open3D)
endif()
if (WITH_ROS)
        add_subdirectory(bindings/ros)
endif()

############################### Install udev rules #######################################
include (${CMAKE_SOURCE_DIR}/cmake/udev-rules-install.cmake)

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
