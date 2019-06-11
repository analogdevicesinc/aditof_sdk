cmake_minimum_required(VERSION 2.8)
project(aditof)

add_definitions(-DSDK_EXPORTS)

find_package(glog 0.3.5 REQUIRED)

file(GLOB SOURCES
    ${CMAKE_SOURCE_DIR}/sdk/lib/*.cpp
)

file(GLOB HEADERS
    ${CMAKE_SOURCE_DIR}/sdk/include/aditof/*.h
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
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/windows/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/windows/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/windows/*.h)
elseif( APPLE )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/macos/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/macos/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/macos/*.h)
elseif( DRAGONBOARD )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/dragonboard/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/dragonboard/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/dragonboard/*.h)
elseif ( UNIX )
    file(GLOB PLATFORM_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/*.c)
    file(GLOB PLATFORM_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/*.h)
endif()

if ( NOT DRAGONBOARD )
    list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/lib/local_device.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_include_directories(${PROJECT_NAME} PRIVATE lib/)

set_target_properties( ${PROJECT_NAME} PROPERTIES VERSION ${VERSION} SOVERSION 1.0 CXX_STANDARD 11)
target_link_libraries( ${PROJECT_NAME} PUBLIC glog::glog)

if( APPLE)
		find_library( LIBUVC_LIB libuvc.dylib /usr/local/lib/ )
		if( NOT LIBUVC_LIB)
			MESSAGE( SEND_ERROR "Cannot find the libuvc library")
		endif( NOT LIBUVC_LIB)
                target_link_libraries( ${PROJECT_NAME} PRIVATE ${LIBUVC_LIB})
endif( APPLE)

