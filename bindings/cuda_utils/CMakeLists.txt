
cmake_minimum_required(VERSION 3.9)
project (aditof_cuda LANGUAGES CXX CUDA)
find_package(CUDA REQUIRED)
IF(CUDA_FOUND)
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

ENDIF()
