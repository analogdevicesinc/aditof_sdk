cmake_minimum_required(VERSION 2.8)
project(examples)

add_subdirectory(aditof-demo)
add_subdirectory(first-frame)

include(GNUInstallDirs)

install(PROGRAMS
    $<TARGET_FILE:aditof-demo>
    $<TARGET_FILE:first-frame>
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)


if(WIN32)
    include(FindOpenSSL)
    add_custom_target(copy-dll-example
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:aditof-demo>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:first-frame>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:aditof-demo>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:OpenSSL::SSL> $<TARGET_FILE_DIR:first-frame>
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
