#!/bin/bash

project_dir=$1
pushd ${project_dir}

GLOG_INSTALL_DIR="/aditof-deps/installed/glog"
PROTOBUF_INSTALL_DIR="/aditof-deps/installed/protobuf"
WEBSOCKETS_INSTALL_DIR="/aditof-deps/installed/websockets"
OPENCV_INSTALL_DIR="/aditof-deps/installed/opencv"
NUM_JOBS=4
ARGS="$2"

mkdir -p build
pushd build
cmake .. ${ARGS} -DCMAKE_PREFIX_PATH="${GLOG_INSTALL_DIR};${PROTOBUF_INSTALL_DIR};${WEBSOCKETS_INSTALL_DIR};${OPENCV_INSTALL_DIR}"
make -j${NUM_JOBS}
popd #build

popd # ${project_dir}
