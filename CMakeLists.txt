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
set(ADITOF_VERSION_MAJOR 3)
set(ADITOF_VERSION_MINOR 1)
set(ADITOF_VERSION_PATCH 0)
set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

set(ADITOF_VERSION_UVC_MAJOR 3)
set(ADITOF_VERSION_UVC_MINOR 1)
set(VERSION_UVC "${ADITOF_VERSION_UVC_MAJOR}.${ADITOF_VERSION_UVC_MINOR}")

set(ADITOF_VERSION_SERVER_MAJOR 3)
set(ADITOF_VERSION_SERVER_MINOR 1)
set(VERSION_SERVER "${ADITOF_VERSION_SERVER_MAJOR}.${ADITOF_VERSION_SERVER_MINOR}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)
option(WITH_OPENCV "Build opencv bindings?" OFF)
option(WITH_MATLAB "Build matlab bindings?" OFF)
option(WITH_OPEN3D "Build open3d bindings?" OFF)
option(WITH_ROS "Build ros bindings?" OFF)
option(WITH_NETWORK "Build network interface?" ON)
option(USE_FXTOF1 "Use FXTOF1 camera?" OFF)
option(USE_3D_SMART "Use 3D Smart camera?" OFF)
option(WITH_TOOLS "Build tools?" OFF)
option(IGNORE_TARGET_VERSION "Ignore sdk version on target?" OFF)
option(WITH_JS "Build JavaScript bindings?" OFF)

# Set camera flags
if (USE_FXTOF1)
        add_definitions(-DFXTOF1)
endif()

# Set jetson platform by default for 3D smart camera
if (USE_3D_SMART)
        add_definitions(-DSMART_3D)
        set(JETSON 1)
endif()

# Check whether we configure things for remote or we're on target
set(SELECTED_PLATFORMS 0)
set(ALL_PLATFORMS DRAGONBOARD;RASPBERRYPI;JETSON;XAVIER;XAVIERNX;TOYBRICK)
foreach(platform IN LISTS ALL_PLATFORMS)
    if(${platform})
        math(EXPR SELECTED_PLATFORMS "${SELECTED_PLATFORMS}+1")
    endif()
endforeach()

set(ON_TARGET FALSE)
if(${SELECTED_PLATFORMS} GREATER 1)
        message(FATAL_ERROR "More than one platform was selected. \nPlease select a single platform. CMake will exit.")
elseif(${SELECTED_PLATFORMS} EQUAL 1)
    add_definitions(-DTARGET)
    set(ON_TARGET TRUE)
endif()

# Throw error if more than one camera is selected when running on target
if (ON_TARGET)
    set(SELECTED_CAMERAS 0)
    set(ALL_CAM_OPTIONS;USE_FXTOF1;USE_3D_SMART)
    foreach(option IN LISTS ALL_CAM_OPTIONS)
    if(${option})
        math(EXPR SELECTED_CAMERAS "${SELECTED_CAMERAS}+1")
    endif()
    endforeach()

    if(${SELECTED_CAMERAS} GREATER 1)
    message(FATAL_ERROR "More than one camera was selected. \nPlease select a single camera. CMake will exit.")
    endif()

    if(WITH_JS)
        message(FATAL_ERROR "Target does not support JavaScript bindings.")
    endif()
endif()

if (WITH_JS)
    set(WITH_NETWORK OFF)
    set(WITH_EXAMPLES OFF)
    add_definitions(-DJS_BINDINGS)
endif()

if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

include(FindGit OPTIONAL)
if (GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_VARIABLE ADITOFSDK_GIT_REPO
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)

	if ("${ADITOFSDK_GIT_REPO}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_COMMIT
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_BRANCH
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	endif()
endif()

add_definitions(-DADITOFSDK_GIT_COMMIT="${ADITOFSDK_GIT_COMMIT}")
add_definitions(-DADITOFSDK_GIT_BRANCH="${ADITOFSDK_GIT_BRANCH}")

add_subdirectory(sdk)
add_subdirectory(apps)

if(WITH_TOOLS)
        if (MSVC)
                add_subdirectory(tools/deps/wingetopt)
        endif(MSVC)
        add_subdirectory(tools/eeprom-tool)
        add_subdirectory(tools/test-fxtof1)
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
