#!/bin/bash

apt update
apt install -y sudo

sudo apt install -y build-essential cmake python-dev python3-dev \
        libopencv-contrib-dev libopencv-dev libssl-dev

PROJECT=$1
pushd ${PROJECT}

GLOG_INSTALL_DIR="$PWD/deps/installed/glog"
PROTOBUF_INSTALL_DIR="$PWD/deps/installed/protobuf"
WEBSOCKETS_INSTALL_DIR="$PWD/deps/installed/websockets"

. ci/travis/lib.sh

build_and_install_glog "deps/glog" ${GLOG_INSTALL_DIR}
build_and_install_protobuf "deps/protobuf" ${PROTOBUF_INSTALL_DIR}
build_and_install_websockets "deps/libwebsockets" ${WEBSOCKETS_INSTALL_DIR}

mkdir -p build
pushd build
cmake .. -DDRAGONBOARD=TRUE -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="${GLOG_INSTALL_DIR};${PROTOBUF_INSTALL_DIR};${WEBSOCKETS_INSTALL_DIR}"
make -j${NUM_JOBS}
popd #build

popd # ${PROJECT}
