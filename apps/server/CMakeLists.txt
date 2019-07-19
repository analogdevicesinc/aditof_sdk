cmake_minimum_required(VERSION 2.8)
project(aditof-server)

find_package(PkgConfig)
pkg_check_modules(LIB_WEBSOCKETS REQUIRED libwebsockets)

find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS buffer.proto)

get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

add_executable(${PROJECT_NAME} server.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} PRIVATE aditof protobuf ${LIB_WEBSOCKETS_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${GENERATED_PROTO_FILES_DIR} ${LIB_WEBSOCKETS_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES
                                         CXX_STANDARD 11
                                         LINK_FLAGS "-L${LIB_WEBSOCKETS_LIBRARY_DIRS}"
)
