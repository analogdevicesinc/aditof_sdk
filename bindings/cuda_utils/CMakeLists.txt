
cmake_minimum_required(VERSION 3.9)
project (aditof_cuda LANGUAGES CXX CUDA)
find_package(CUDA REQUIRED)
IF(CUDA_FOUND)
    #ONLY FOR DEBUG
    add_definitions(-DDEBUG=1)

    add_definitions(-DPATH_TO_CNN_JSON="${CMAKE_SOURCE_DIR}/json_model/")

    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -dlink)
    cuda_compile(cuda_utils cuda_utils.cu)
    add_library(aditof_cuda SHARED
        src/cuda_utils.cu
        )
    set_target_properties(aditof_cuda PROPERTIES
                        PUBLIC_HEADER include/cuda_utils.h
                        )
    install(TARGETS  aditof_cuda
            LIBRARY DESTINATION ../../cuda_lib/
            PUBLIC_HEADER DESTINATION ../../cuda_lib/)
    install(FILES ${CMAKE_BINARY_DIR}/aditof_cuda.pc
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

    target_compile_options(aditof_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        -arch=sm_53
    >)

ENDIF()
