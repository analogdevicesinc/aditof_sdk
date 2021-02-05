#!/bin/bash

set -ex

#we need to export all those variables from a script because of pwd
setup_absolute_paths() {
    export WORK_DIR=$(pwd) 
    export BUILD_DIR="${WORK_DIR}/build"
    export DEPS_DIR="${BUILD_DIR}/deps"
}

setup_absolute_paths