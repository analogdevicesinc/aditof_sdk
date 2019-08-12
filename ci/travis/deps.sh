#!/bin/bash

. ci/travis/lib.sh

set -e

. ci/travis/lib.sh

deps_default() {
    install_glog
    install_protobuf
    install_websockets
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

    # Clone glog now so we don't need git inside of
    # the docker container
    pushd "${DEPS_DIR}"
    [ -d "glog" ] || {
       git clone https://github.com/google/glog
    }
    pushd glog
    git checkout tags/v0.3.5
    popd
    popd
}

deps_${BUILD_TYPE:-default}
