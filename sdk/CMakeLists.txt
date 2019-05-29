cmake_minimum_required(VERSION 2.8)
project(aditof)

add_definitions(-DSDK_EXPORTS)

find_package(glog 0.3.5 REQUIRED)

set(SOURCES
        src/frame_operations.cpp
        src/frame_impl.cpp
        src/frame.cpp
        src/camera_impl.cpp
        src/camera.cpp
        src/system_impl.cpp
        src/system.cpp
        src/device_enumerator_impl.cpp
        src/device_enumerator_factory.cpp
        src/usb_device.cpp
        src/ethernet_device.cpp
        src/device_factory.cpp
        src/calibration.cpp

        src/status_definitions.h
        src/frame_definitions.h
        src/frame_operations.h
        src/frame_impl.h
        src/frame.h
        src/camera_definitions.h
        src/camera_impl.h
        src/camera.h
        src/system_impl.h
        src/system.h
        src/device_construction_data.h
        src/device_enumerator_interface.h
        src/device_enumerator_impl.h
        src/device_enumerator_factory.h
        src/device_interface.h
        src/usb_device.h
        src/local_device.h
        src/ethernet_device.h
        src/device_factory.h
        src/sdk_exports.h
        src/calibration.h
)

if( WIN32 )
	set(OS_NAME Windows)
elseif( APPLE )
	set(OS_NAME MacOS)
elseif( DRAGONBOARD )
	set(OS_NAME Dragonboard)
elseif ( UNIX )
	set(OS_NAME Linux)
else()
	set(OS_NAME Unknown)
endif()

# Platform specific sources and headers
if( WIN32 )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/windows/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/windows/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/windows/*.h)
elseif( APPLE )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/macos/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/macos/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/macos/*.h)
elseif( DRAGONBOARD )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dragonboard/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dragonboard/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dragonboard/*.h)
elseif ( UNIX )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linux/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linux/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/linux/*.h)
endif()

if ( NOT DRAGONBOARD )
    list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/local_device.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
)

target_include_directories(${PROJECT_NAME} PUBLIC src/)

set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION 1.0.0 SOVERSION 1.0 CXX_STANDARD 11)
target_link_libraries( ${PROJECT_NAME} PUBLIC glog::glog)

if( APPLE)
		find_library( LIBUVC_LIB libuvc.dylib /usr/local/lib/ )
		if( NOT LIBUVC_LIB)
			MESSAGE( SEND_ERROR "Cannot find the libuvc library")
		endif( NOT LIBUVC_LIB)
                target_link_libraries( ${PROJECT_NAME} PRIVATE ${LIBUVC_LIB})
endif( APPLE)

