#!/bin/bash

set -e

. ci/travis/lib.sh

build_default() {
    pushd "${TRAVIS_BUILD_DIR}"
    cd build
    sudo make -j${NUM_JOBS}
    popd
}

build_cppcheck() {
    check_cppcheck
}

build_clang_format() {
    check_clangformat
}

build_deploy_doxygen() {
    pushd "${TRAVIS_BUILD_DIR}/doc"
    mkdir build && cd build && cmake .. 
    check_doxygen
    deploy_doxygen
}

build_dragonboard() {
    docker run --rm --privileged multiarch/qemu-user-static:register --reset

    sudo docker run --rm=true \
			-v `pwd`:/aditof_sdk:rw \
			rycus86/arm64v8-debian-qemu \
            /bin/bash -xe /aditof_sdk/ci/travis/inside_debian_docker.sh aditof_sdk
}

build_${BUILD_TYPE:-default}
