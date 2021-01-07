#!/bin/bash

# Build the given dockerfile with the given name

. ci/azure/lib.sh

mkdir -p temp_deps

get_deps_source_code temp_deps

dockername=$1
dockerfilepath=$2

docker build -t ${dockername} -f ${dockerfilepath} .

rm -rf temp_deps
