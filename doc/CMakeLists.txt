cmake_minimum_required(VERSION 2.8)
project(doc)

find_program(DOXYGEN_PATH doxygen)
if (NOT DOXYGEN_PATH)
    message(FATAL_ERROR "doxygen not found!")
endif()

######################### This script variables ###############################
set(SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sdk/include/aditof)
set(DOXY_WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen_doc)

############################# Doc variables ###################################
set(PROJECT_NAME "Time-of-Flight-SDK")
set(DOCUMENTED_FILES "${SOURCES_DIR}/frame.h \\
                      ${SOURCES_DIR}/frame_definitions.h \\
                      ${SOURCES_DIR}/frame_operations.h \\
                      ${SOURCES_DIR}/camera.h \\
                      ${SOURCES_DIR}/camera_definitions.h \\
                      ${SOURCES_DIR}/system.h \\
                      ${SOURCES_DIR}/status_definitions.h \\
                      ${SOURCES_DIR}/connections.h \\
                      ${SOURCES_DIR}/depth_sensor_interface.h \\
                      ${SOURCES_DIR}/sensor_definitions.h \\
                      ${SOURCES_DIR}/storage_interface.h \\
                      ${SOURCES_DIR}/temperature_sensor_interface.h \\
                      ${SOURCES_DIR}/sensor_enumerator_interface.h \\
                      ${SOURCES_DIR}/sensor_enumerator_factory.h \\
                      ${SOURCES_DIR}/target_sdk_definitions.h \\
                      ${SOURCES_DIR}/log_cout.h \\
                      ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.dox
                      "
)

################# Generate Doxygen configuration file #########################
configure_file(
    sdk.doxy.in
    ${DOXY_WORKING_DIR}/sdk-api.doxy
)

################# Custom target to build Doxygen doc ##########################
add_custom_target(doc
    COMMAND ${DOXYGEN_PATH} sdk-api.doxy
    COMMAND ${CMAKE_COMMAND} -E make_directory ${DOXY_WORKING_DIR}/html
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/img ${DOXY_WORKING_DIR}/html/img
    WORKING_DIRECTORY ${DOXY_WORKING_DIR}
)

############### Install the documentation in the system #######################
include(GNUInstallDirs)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doxygen_doc/html/ DESTINATION ${CMAKE_INSTALL_DOCDIR})
