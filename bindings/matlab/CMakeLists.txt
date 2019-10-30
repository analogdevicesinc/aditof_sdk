cmake_minimum_required(VERSION 2.8)

project(aditofadapter)

# TODO: copy tof.imdf

include(../../cmake/FindMatlabWithImageAcquisitionToolbox.cmake)

file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
)

if (CMAKE_COMPILER_IS_GNUCC)
	target_compile_options(${PROJECT_NAME} PUBLIC
		-Wall
		-Wno-unknown-pragmas
		-Werror=return-type
		-Werror=uninitialized
		-Werror=init-self
	)
endif()

# Make sure we have no prefix for our output name
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${Matlab_MWIMAQMEX_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        aditof
        ${Matlab_MX_LIBRARY}
        ${Matlab_MWIMAQMEX_LIBRARY}
)
