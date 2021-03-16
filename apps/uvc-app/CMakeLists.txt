cmake_minimum_required(VERSION 2.8)
project(uvc-gadget)

find_package(Protobuf 3.9.0 REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HRDS uvc_buffer.proto)

get_filename_component(GENERATED_PROTO_FILES_DIR ${PROTO_HRDS} DIRECTORY)

add_executable(${PROJECT_NAME} uvc-gadget.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${Protobuf_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${Protobuf_INCLUDE_DIRS} ${GENERATED_PROTO_FILES_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)




configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config_pipe.sh
               ${CMAKE_CURRENT_BINARY_DIR}/config_pipe.sh
               COPYONLY)
