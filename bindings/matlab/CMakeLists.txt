cmake_minimum_required(VERSION 2.8)

project(aditofadapter)

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

if (MSVC)
        target_compile_options(${PROJECT_NAME} PUBLIC
                /EHs # enable c++ exceptions, yes with extern C functions
                /MD # multi-threaded DLL
                /GS # disable security checks
                /GR # enable run-time type information
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
        $<$<NOT:$<PLATFORM_ID:Windows>>:${Matlab_MX_LIBRARY}> # MX lib only needed on Linux/MacOS
        ${Matlab_MWIMAQMEX_LIBRARY}
)
