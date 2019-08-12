#!/bin/bash

set -e

. ci/travis/lib.sh

get_deps_source_code()
{
    pushd "${DEPS_DIR}"
    [ -d "glog" ] || {
       git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
    }
    [ -d "protobuf" ] || {
       git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
    }
    [ -d "libwebsockets" ] || {
       git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
    }
    popd
}

deps_default() {
    build_and_install_glog "${DEPS_DIR}/glog"
    build_and_install_protobuf "${DEPS_DIR}/protobuf"
    build_and_install_websockets "${DEPS_DIR}/libwebsockets"
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
    sudo apt-get -qq update
	sudo service docker restart
    sudo docker pull rycus86/arm64v8-debian-qemu
}

get_deps_source_code
deps_${BUILD_TYPE:-default}
