#!/bin/bash

apt update
apt install -y sudo

sudo apt install -y build-essential cmake python-dev python3-dev \
        libopencv-contrib-dev libopencv-dev

PROJECT=$1
pushd ${PROJECT}

. ci/travis/lib.sh

build_and_install_glog "deps/glog"
build_and_install_protobuf "deps/protobuf"
build_and_install_websockets "deps/libwebsockets"

mkdir -p build
pushd build
cmake .. -DDRAGONBOARD=TRUE -DWITH_PYTHON=on
make -j${NUM_JOBS}
popd #build

popd # ${PROJECT}
