#!/bin/bash

set -e

. ci/travis/lib.sh

deps_default() {
    get_deps_source_code ${DEPS_DIR}
    build_and_install_glog "${DEPS_DIR}/glog" "${DEPS_DIR}/installed/glog"
    build_and_install_protobuf "${DEPS_DIR}/protobuf" "${DEPS_DIR}/installed/protobuf"
    build_and_install_websockets "${DEPS_DIR}/libwebsockets" "${DEPS_DIR}/installed/websockets"
}

deps_cppcheck() {
    echo_green "Cppcheck version: " `cppcheck --version`
    cppcheck --version
}

deps_clang_format() {
    echo_green "Clang-format version: " `/usr/bin/clang-format-6.0 --version`
}

deps_deploy_doxygen() {
    install_doxygen
    echo_green "Doxygen version: " `doxygen --version`
}

deps_dragonboard() {
    get_deps_source_code
    sudo apt-get -qq update
	sudo service docker restart
    sudo docker pull rycus86/arm64v8-debian-qemu
}

deps_${BUILD_TYPE:-default}
