#!/bin/bash

set -ex

. ci/azure/lib.sh


build_default() {
    
    # setup more deps
    # TODO cache this
    sudo apt-get update
    sudo apt-get install -y build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  
    sudo sh -c 'echo "${DEPS_DIR}/installed/opencv/lib" > /etc/ld.so.conf.d/opencv.conf'
	sudo ldconfig

    # setup compiler
    if [[ "${COMPILER_CXX}" != "" ]]; then export CXX=${COMPILER_CXX}; fi
    if [[ "${COMPILER_CC}" != "" ]]; then export CC=${COMPILER_CC}; fi
    
    mkdir -p ${BUILD_DIR}

    pushd ${BUILD_DIR}
    pwd
    cmake ${DEFAULT_CMAKE_FLAGS} ${EXTRA_CMAKE_FLAGS} -DCMAKE_PREFIX_PATH="${DEPS_DIR}/installed/glog;${DEPS_DIR}/installed/protobuf;${DEPS_DIR}/installed/websockets;${DEPS_DIR}/installed/Open3D;${DEPS_DIR}/installed/opencv" .. 
    make -j${NUM_JOBS}
    popd
}

build_cppcheck() {
    check_cppcheck
}

build_clang_format() {
    check_clangformat
}

build_deploy_doxygen() {
    mkdir -p "${WORK_DIR}/doc"
    pushd "${WORK_DIR}/doc"
    mkdir build && cd build && cmake ..
    check_doxygen
    popd
    
    deploy_doxygen
}

build_docker() {
    run_docker ${DOCKER} /aditof_sdk/ci/azure/inside_docker.sh "${DEFAULT_CMAKE_FLAGS} ${EXTRA_CMAKE_FLAGS}"
}

build_ros(){
    pushd "${TRAVIS_BUILD_DIR}"
    cd build
    sudo cmake --build . --target install
    cmake --build . --target aditof_ros_package
    popd
}

if [[ "${DOCKER}" != "" ]]; then export BUILD_TYPE="docker"; fi

build_${BUILD_TYPE:-default}
