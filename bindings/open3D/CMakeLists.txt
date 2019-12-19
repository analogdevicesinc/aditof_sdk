cmake_minimum_required(VERSION 2.8)

if( DRAGONBOARD OR RASPBERRYPI )
    message(WARNING "Open3D bindings are not available on target!")
else()
    # Install the helper header for open3d in include/aditof
    include(GNUInstallDirs)
    install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/aditof_open3d.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/aditof
    )

    if(WIN32)
        find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
    else()
        find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
        list(APPEND Open3D_LIBRARIES dl)
    endif()

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")

    # Set OS-specific things here
    if(WIN32)
    elseif(APPLE)
    elseif(UNIX)
        add_compile_options(-Wno-deprecated-declarations)
        add_compile_options(-Wno-unused-result)
    endif(WIN32)

    # Open3D
    if (Open3D_FOUND)
        message(STATUS "Found Open3D ${Open3D_VERSION}")

        # Hot fix windows dll not found issue, assumming we're using the Release build
        option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)
        if(WIN32 AND BUILD_SHARED_LIBS)
            message("Will copy Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                            COMMAND ${CMAKE_COMMAND} -E copy
                                    ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                    ${CMAKE_CURRENT_BINARY_DIR}/Release)
        endif()

    else ()
        message(SEND_ERROR "Open3D not found")
    endif ()

    add_subdirectory(showPointCloud)
endif()
