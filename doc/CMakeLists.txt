cmake_minimum_required(VERSION 2.8)
project(doc)

find_program(DOXYGEN_PATH doxygen)

if (NOT DOXYGEN_PATH)
    message(FATAL_ERROR "doxygen not found!")
endif()

set(SOURCES_DIR ${CMAKE_SOURCE_DIR}/sdk/src)

############################ Doc variables ####################################
set(PROJECT_NAME "Time-of-Flight-SDK")
set(DOCUMENTED_FILES "${SOURCES_DIR}/frame.h \\
                      ${SOURCES_DIR}/frame_definitions.h \\
                      ${SOURCES_DIR}/camera.h \\
                      ${SOURCES_DIR}/camera_definitions.h \\
                      ${SOURCES_DIR}/system.h \\
                      ${SOURCES_DIR}/status_definitions.h
                      "
)

################# Generate Doxygen configuration file #########################
configure_file(
    sdk.doxy.in
    ${CMAKE_BINARY_DIR}/doxygen_doc/sdk-api.doxy
)
