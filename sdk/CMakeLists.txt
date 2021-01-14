cmake_minimum_required(VERSION 2.8)
project(aditof)

add_definitions(-DSDK_EXPORTS)
find_package(glog 0.3.5 REQUIRED)

if ( WITH_NETWORK )
    find_package(Protobuf 3.9.0 REQUIRED)
    find_package(Libwebsockets REQUIRED)

    protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS src/connections/network/buffer.proto)
    get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)
endif()

file(GLOB SOURCES
    ${CMAKE_SOURCE_DIR}/sdk/src/*.cpp
)

file(GLOB HEADERS
    ${CMAKE_SOURCE_DIR}/sdk/include/aditof/*.h
)

if (USE_CHICONY)
    if (NOT RASPBERRYPI)
        message( FATAL_ERROR "USE_CHICONY only works when on RaspberryPI. CMake will exit." )
    endif()
    set(CAMERA_DIR "cameras/chicony-006")
elseif(USE_FXTOF1)
    set(CAMERA_DIR "cameras/ad-fxtof1-ebz")
else ()
    set(CAMERA_DIR "cameras/ad-96tof1-ebz")
endif()    

if( WIN32 )
        set(OS_SPECIFIC_DIR "connections/usb/windows")
elseif( APPLE )
        set(OS_SPECIFIC_DIR "connections/usb/macos")
elseif( DRAGONBOARD )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR dragonboard)
	add_definitions(-DDRAGONBOARD)
elseif ( RASPBERRYPI )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR raspberrypi)
	add_definitions(-DRASPBERRYPI)
elseif( JETSON )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR jetson)
        add_definitions(-DJETSON)
elseif( XAVIER )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR xavier_agx)
	    add_definitions(-DXAVIER)
elseif( XAVIERNX )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR xavier_nx)
	    add_definitions(-DXAVIERNX)
elseif( TOYBRICK )
        set(OS_SPECIFIC_DIR "connections/target")
        set(TARGET_SPECIFIC_DIR toybrick)
	    add_definitions(-DTOYBRICK)
elseif ( UNIX )
        set(OS_SPECIFIC_DIR "connections/usb/linux")
else()
    message(FATAL_ERROR "Platform not supported")
endif()

if (NOT APPLE AND NOT WIN32)
    option(RUN_LDCONFIG "Run ldconfig after install." ON)
endif()

if ( USE_CHICONY AND USE_FXTOF1)
    message( FATAL_ERROR "USE_CHICONY and USE_FXTOF1 are both selected. \nPlease select a single camera. CMake will exit." )
endif()

# Platform specific sources and headers
file(GLOB PLATFORM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/${TARGET_SPECIFIC_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/usb/*.cpp
)
file(GLOB PLATFORM_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${OS_SPECIFIC_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/usb/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${CAMERA_DIR}/${TARGET_SPECIFIC_DIR}/*.h
)

file(GLOB NETWORK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/network/*.cpp
)
file(GLOB NETWORK_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/connections/network/*.h
)

# Create target and set properties
add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
    ${PLATFORM_SOURCES}
    ${PLATFORM_HEADERS}
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_SOURCES}>
    $<$<BOOL:${WITH_NETWORK}>:${NETWORK_HEADERS}>
    $<$<BOOL:${WITH_NETWORK}>:${PROTO_SRCS}>
    $<$<BOOL:${WITH_NETWORK}>:${PROTO_HDRS}>
)

# Add alias to aditof as aditof::aditof
add_library(aditof::aditof ALIAS aditof)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        src/
        src/${OS_SPECIFIC_DIR}/${TARGET_SPECIFIC_DIR}
        src/${CAMERA_DIR}/
        ${Protobuf_INCLUDE_DIRS}
        ${GENERATED_PROTO_FILES_DIR}
        ${LIBWEBSOCKETS_INCLUDE_DIRS}
        ${OPENSSL_INCLUDE_DIRS}
)

set_target_properties( ${PROJECT_NAME} PROPERTIES
                       VERSION ${VERSION}
                       SOVERSION 1.0
                       CXX_STANDARD 11
)

if ( WITH_NETWORK )
    target_link_libraries( ${PROJECT_NAME}
        PUBLIC
            glog::glog
        PRIVATE
            ${Protobuf_LIBRARIES}
            ${LIBWEBSOCKETS_LIBRARIES}
    )
else()
    target_link_libraries( ${PROJECT_NAME}
        PUBLIC
            glog::glog
    )
endif()

if (USE_CHICONY AND RASPBERRYPI)
    if (NOT DEFINED CHICONY_EEPROM_SO)
        set(CHICONY_EEPROM_SO "${CMAKE_SOURCE_DIR}/misc/chicony/libChicony_EEPROM.so")
    endif()

    add_library(chicony-eeprom SHARED IMPORTED)
    set_target_properties(chicony-eeprom PROPERTIES
        IMPORTED_LOCATION ${CHICONY_EEPROM_SO}
        IMPORTED_NO_SONAME 1
    )

    target_link_libraries(${PROJECT_NAME} PRIVATE chicony-eeprom)
endif()

if (CMAKE_COMPILER_IS_GNUCC)
    target_compile_options(${PROJECT_NAME} PUBLIC
            -Wall
            -Wno-unknown-pragmas
            -Werror=return-type
            -Werror=uninitialized
            -Werror=init-self
    )
endif()

if ( RASPBERRYPI )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -mfpu=neon -latomic")
    include (${CMAKE_SOURCE_DIR}/cmake/raspberrypi-revision-config.cmake)
endif()

if ( JETSON )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -latomic")
endif()

if( APPLE )
    find_library( LIBUVC_LIB libuvc.dylib /usr/local/lib/ )
    if( NOT LIBUVC_LIB)
        MESSAGE( SEND_ERROR "Cannot find the libuvc library")
    endif( NOT LIBUVC_LIB)
    target_link_libraries( ${PROJECT_NAME} PRIVATE ${LIBUVC_LIB})
endif( APPLE )

# includes install paths
include(GNUInstallDirs)

# install path for the aditof-config.cmake and aditof-config-version.cmake
set(INSTALL_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/aditof)

# export targets for the config cmake to use
install(TARGETS aditof
    EXPORT aditof-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT aditof-targets
    FILE
        aditof-targets.cmake
    NAMESPACE
        aditof::
    DESTINATION
        ${INSTALL_CONFIG_DIR}
)

# install the include folder
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# include helper functions to write the config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config-version.cmake
    VERSION ${VERSION}
    COMPATIBILITY AnyNewerVersion
)

set(CONFIG_INCLUDE_DIRS "${CMAKE_PREFIX_PATH}/${CMAKE_INSTALL_INCLUDEDIR}")

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/aditof-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIG_DIR}
)

# install the files in order to be used by other projects
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/aditof-config-version.cmake
    DESTINATION ${INSTALL_CONFIG_DIR}
)
