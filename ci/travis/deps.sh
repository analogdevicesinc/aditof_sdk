#!/bin/bash

. ci/travis/lib.sh

set -e

. ci/travis/lib.sh

deps_default() {
    install_glog
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

deps_${BUILD_TYPE:-default}
