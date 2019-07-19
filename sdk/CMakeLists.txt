cmake_minimum_required(VERSION 2.8)
project(aditof)

add_definitions(-DSDK_EXPORTS)

find_package(glog 0.3.5 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig)
pkg_check_modules(LIB_WEBSOCKETS REQUIRED libwebsockets)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS src/buffer.proto)
get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

file(GLOB SOURCES
    ${CMAKE_SOURCE_DIR}/sdk/src/*.cpp
)

file(GLOB HEADERS
    ${CMAKE_SOURCE_DIR}/sdk/include/aditof/*.h
)

if( WIN32 )
        set(OS_SPECIFIC_DIR windows)
elseif( APPLE )
        set(OS_SPECIFIC_DIR macos)
elseif( DRAGONBOARD )
        set(OS_SPECIFIC_DIR target)
        set(TARGET_SPECIFIC_DIR dragonboard)
elseif ( RASPBERRYPI )
        set(OS_SPECIFIC_DIR target)
        set(TARGET_SPECIFIC_DIR raspberrypi)
elseif ( UNIX )
        set(OS_SPECIFIC_DIR linux)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

# Platform specific sources and headers
file(GLOB PLATFORM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/${TARGET_SPECIFIC_DIR}/*.cpp
)
file(GLOB PLATFORM_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/${TARGET_SPECIFIC_DIR}/*.h
)

if ( "${OS_SPECIFIC_DIR}" STREQUAL "target" )
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/local_device.cpp)
endif()

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_include_directories(${PROJECT_NAME} PRIVATE
    src/
    src/${OS_SPECIFIC_DIR}/${TARGET_SPECIFIC_DIR}
    ${GENERATED_PROTO_FILES_DIR}
    ${LIB_WEBSOCKETS_INCLUDE_DIRS}
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
                       VERSION ${VERSION}
                       SOVERSION 1.0
                       CXX_STANDARD 11
                       LINK_FLAGS "-L${LIB_WEBSOCKETS_LIBRARY_DIRS}"
)
target_link_libraries( ${PROJECT_NAME} PUBLIC
                       glog::glog
                       protobuf
                       ${LIB_WEBSOCKETS_LIBRARIES}
)

if ( RASPBERRYPI )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -mfpu=neon")
endif()

if( APPLE )
    find_library( LIBUVC_LIB libuvc.dylib /usr/local/lib/ )
    if( NOT LIBUVC_LIB)
        MESSAGE( SEND_ERROR "Cannot find the libuvc library")
    endif( NOT LIBUVC_LIB)
    target_link_libraries( ${PROJECT_NAME} PRIVATE ${LIBUVC_LIB})
endif( APPLE )
