#!/bin/bash

apt update
apt install sudo

sudo apt install -y build-essential cmake python-dev python3-dev \
        libopencv-contrib-dev libopencv-dev

PROJECT=$1
pushd ${PROJECT}

. ci/travis/lib.sh

pushd deps/glog
mkdir -p build
pushd build
cmake -DWITH_GFLAGS=off ..
make -j${NUM_JOBS}
sudo make install
popd # build
popd # deps/glog

mkdir -p build
pushd build
cmake .. -DDRAGONBOARD=TRUE -DWITH_PYTHON=on
make -j${NUM_JOBS}
popd #build

popd # ${PROJECT}
