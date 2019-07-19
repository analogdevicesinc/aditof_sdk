cmake_minimum_required(VERSION 2.8)
project(adi_tof_project)

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 1)
set(ADITOF_VERSION_MINOR 0)
set(ADITOF_VERSION_PATCH 0)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)

add_subdirectory(sdk)
if (WITH_EXAMPLES)
        add_subdirectory(examples)
endif()
if (WITH_DOC)
        add_subdirectory(doc)
endif()
if (WITH_PYTHON)
        add_subdirectory(bindings/python)
endif()
#if (DRAGONBOARD)
        add_subdirectory(apps/server)
#endif()

############################### Installer #######################################
configure_file(aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
