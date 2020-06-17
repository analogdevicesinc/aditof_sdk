#!/bin/bash

NUM_JOBS=4

echo_red() { printf "\033[1;31m$*\033[m\n"; }
echo_green() { printf "\033[1;32m$*\033[m\n"; }

############################################################################
# Check if the file passed as arguments is ignored or not by clang format
############################################################################
is_not_ignored() {
    local file="$1"

    fileData=`cat .clangformatignore`

    for entry in $fileData; do
        if [ -d "${entry}" ]; then
            pushd ${entry}
            fileName=`basename ${file}`
            found=$(find -name ${fileName} | wc -l)
            if [ ${found} -gt 0 ]; then
                popd
                return 1
            else
                popd
            fi
        else
            if [ -f "${entry}" ]; then
                if [ "${file}" == "${entry}" ]; then
                    return 1
                fi
            fi
        fi
    done;
    return 0
}


############################################################################
# Check if the file given as input has .h or .cpp extension
############################################################################
is_source_file() {
    local file="$1"

    EXTENSIONS=".h .cpp"

    for extension in $EXTENSIONS; do
        [[ "${file: -2}" == "$extension" || "${file: -4}" == "$extension" ]] && return 0
    done;

    return 1
}

############################################################################
# Check if the files modified in the current commit / commit range respect
# the coding style defined in the .clang-format file
############################################################################
check_clangformat() {

    COMMIT_RANGE=$TRAVIS_COMMIT_RANGE

    if [ -z "$TRAVIS_PULL_REQUEST_SHA" ]
    then
        COMMIT_RANGE=HEAD~1
    fi

    git diff --name-only --diff-filter=d $COMMIT_RANGE | while read -r file; do
        if is_source_file "$file" && is_not_ignored "$file"
        then
            /usr/bin/clang-format-6.0 -i "$file"
        fi

    done;

    git diff --exit-code || {
        echo_red "The code is not properly formatted."
        exit 1
    }

}

############################################################################
# Run the cpp static analysis tool on the sdk
############################################################################
check_cppcheck() {
   cppcheck --quiet --force --enable=warning,performance,portability,style --std=c++11 sdk/ apps/
}

############################################################################
# Check if the documentation will be generated w/o warnings or errors
############################################################################
check_doxygen() {
    pushd ${TRAVIS_BUILD_DIR}/doc
    (cd build && ! make doc 2>&1 | grep -E "warning|error") || {
        echo_red "Documentation incomplete or errors in the generation of it have occured!"
        exit 1
    }
    popd
    echo_green "Documentation was generated successfully!"
}

############################################################################
# If the current build is not a pull request and it is on master the 
# documentation will be pushed to the gh-pages branch if changes occurred
# since the last version that was pushed
############################################################################
deploy_doxygen() {
    if [[ "${TRAVIS_PULL_REQUEST}" == "false" && "${TRAVIS_BRANCH}" == "master" ]]
    then
        pushd ${TRAVIS_BUILD_DIR}/doc
        git clone https://github.com/${TRAVIS_REPO_SLUG} --depth 1 --branch=gh-pages doc/html &>/dev/null

        pushd doc/html
        rm -rf *
        popd
        
        cp -R build/doxygen_doc/html/* doc/html/

        pushd doc/html
        CURRENT_COMMIT=$(git log -1 --pretty=%B)
        if [[ ${CURRENT_COMMIT:(-7)} != ${TRAVIS_COMMIT:0:7} ]]
        then
            git add --all .
            git commit --allow-empty --amend -m "Update documentation to ${TRAVIS_COMMIT:0:7}"
            git push https://${GITHUB_DOC_TOKEN}@github.com/${TRAVIS_REPO_SLUG} gh-pages -f &>/dev/null
        else
            echo_green "Documentation already up to date!"
        fi
        popd
    else
        echo_green "Documentation will be updated when this commit gets on master!"
    fi
}

############################################################################
# Build and install v0.3.5 of glog from the specified repository
############################################################################
build_and_install_glog() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_0_3_5

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake .. -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}

############################################################################
# Build and install v3.9.0 of protobuf from the specified repository
############################################################################
build_and_install_protobuf() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_3_9_0

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake ../cmake/ -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}

############################################################################
# Build and install v3.1.0 of libwebsockets from the specified repository
############################################################################
build_and_install_websockets() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_3_1_0

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake .. -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ${EXTRA_CMAKE_OPTIONS}
    make -j${NUM_JOBS}
    sudo make install
    popd
}

############################################################################
# Build and install opencv from the specified repository
############################################################################
build_and_install_opencv() {
    # OpenCV version may be custom so we replace the . with _ to make
    # he build folder. example: 3.4.1 => 3_4_1
    OPENCV_BUILD_VERSION=`echo ${OPENCV} | sed -r 's/[.]/_/g'`
    REPO_DIR=$1
    INSTALL_DIR=$2
    BUILD_DIR=${REPO_DIR}/build_${OPENCV_BUILD_VERSION}

    # Install some packages requiered for OpenCV
    sudo apt-get install -y build-essential libgtk2.0-dev pkg-config libavcodec-dev \
        libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev \
        libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev binutils

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} -D WITH_TBB=OFF -D WITH_IPP=OFF -D BUILD_NEW_PYTHON_SUPPORT=OFF -D WITH_V4L=OFF -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -DCPU_DISPATCH= ..
    make -j${NUM_JOBS}
    sudo make install

	sudo sh -c 'echo "${INSTALL_DIR}/lib" > /etc/ld.so.conf.d/opencv.conf'
	sudo ldconfig
    
    popd
}

############################################################################
# Build and install open3d from the specified repository
############################################################################
build_and_install_open3d() {
    REPO_DIR=$1
    INSTALL_DIR=$2
    EXTRA_CMAKE_OPTIONS=$3
    BUILD_DIR=${REPO_DIR}/build_3_1_0

    chmod +x ${REPO_DIR}/util/scripts/install-deps-ubuntu.sh
    bash ${REPO_DIR}/util/scripts/install-deps-ubuntu.sh "assume-yes"

    mkdir -p ${BUILD_DIR}
    pushd ${BUILD_DIR}
    cmake -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DBUILD_PYBIND11=off -DBUILD_PYTHON_MODULE=off -DGLIBCXX_USE_CXX11_ABI=on ..
    make -j${NUM_JOBS}
    sudo make install
}

############################################################################
# Install the latest version of doxygen in the /deps folder
############################################################################
install_doxygen() {
    DOXYGEN_URL="wget https://sourceforge.net/projects/doxygen/files/rel-1.8.15/doxygen-1.8.15.src.tar.gz"
    pushd ${DEPS_DIR}
    [ -d "doxygen" ] || {
        mkdir doxygen && wget --quiet -O - ${DOXYGEN_URL} | tar --strip-components=1 -xz -C doxygen
    }
    pushd doxygen
    mkdir -p build && cd build
    cmake ..
    make -j${NUM_JOBS}
    sudo make install
    popd
    popd
}

############################################################################
# Get source code for dependencies: glog, protobuf, libwebsockets
############################################################################
get_deps_source_code() {
    CLONE_DIRECTORY=$1
    pushd "${CLONE_DIRECTORY}"

    # use opencv 3.4.1 by default if no other version is specified
    if [[ "${OPENCV}" == "" ]]; then
        export OPENCV="3.4.1"
    fi

    [ -d "glog" ] || {
       git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
    }
    [ -d "protobuf" ] || {
       git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
    }
    [ -d "libwebsockets" ] || {
       git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
    }
    if [[ ${CMAKE_OPTIONS} == *"WITH_OPENCV=on"* ]]; then
        [ -d "opencv-${OPENCV}" ] || {
            curl -sL https://github.com/Itseez/opencv/archive/${OPENCV}.zip > opencv.zip
	        unzip -q opencv.zip
        }
    fi
    if [[ ${CMAKE_OPTIONS} == *"WITH_OPEN3D=on"* ]]; then
        [ -d "Open3D" ] || {
            git clone --recursive --branch v0.9.0 --depth 1 https://github.com/intel-isl/Open3D.git
        }
    fi

    popd
}

############################################################################
# Pull the docker given as argument from docker hub
############################################################################
pull_docker() {
    docker=$1

    sudo apt-get -qq update
	sudo service docker restart

    docker run --rm --privileged multiarch/qemu-user-static:register --reset

    docker_file=${DEPS_DIR}/docker-image.tar
    if [[ -f ${docker_file} ]]; then
        echo_green "Found ${docker} in cache!"
        docker load -i ${docker_file}
    else
        echo_green "Pulling ${docker} from docker hub!"
        docker pull ${docker}
        docker save -o ${docker_file} ${docker}
    fi
}

############################################################################
# Run the script given as argument 2 inside the docker given as argument 1
# with the given arguments 3
############################################################################
run_docker() {
    docker=$1
    script=$2
    script_args=$3

    docker run --rm --privileged multiarch/qemu-user-static:register --reset

    sudo docker run --rm=true \
			-v `pwd`:/aditof_sdk:rw \
			${docker} \
            /bin/bash -xe ${script} /aditof_sdk ${script_args}
}
