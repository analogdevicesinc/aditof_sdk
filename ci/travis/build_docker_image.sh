#!/bin/bash

# Build the given dockerfile with the given name

. ci/travis/lib.sh

mkdir -p temp_deps

export CMAKE_OPTIONS="-DWITH_OPENCV=on"
get_deps_source_code temp_deps

dockername=$1
dockerfilepath=$2

docker build -t ${dockername} -f ${dockerfilepath} .

rm -r temp_deps
