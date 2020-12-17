#!/bin/bash


source_dir=$(cd "$(dirname "$0")/../.."; pwd)

. "${source_dir}"/ci/travis/lib.sh

print_help() {
        echo "./setup [OPTIONS]"
        echo ""
        echo "-h|--help"
        echo "        Print a usage message briefly summarizing the command line options available, then exit."
        echo "-y|--yes"
        echo "        Automatic yes to prompts."
        echo "-b|--buildir"
        echo "        Specify the build directory of the SDK."
        echo "-d|--depsdir"
        echo "        Specify the directory where the dependencies will be downloaded."
        echo "-i|--depsinstalldir"
        echo "        Specify the directory where the dependencies will be installed."
        echo "-j|--jobs"
        echo "        Specify the number of jobs to run in parallel when building dependencies and the SDK"
        echo "-bo|--buildopencv"
        echo "        Build and install the minimum required version of opencv for the dnn example (3.4.1)"
        echo "-op|--opencvpath"
        echo "        Path to opencv installation "
        echo "-ur|--udevrules"
        echo "        Install udev rules for devices permissions"
        echo "-na|--no-apt"
        echo "        Do not install apt packages"
        echo "-es|--enable-server"
        echo "        Enable the server service"
        echo ""
}

install_required_packages() {
        sudo apt install -y build-essential cmake python-dev python3-dev \
        libssl-dev git
}

get_opencv_source_code() {
        pushd "$1"

        [ -d "opencv" ] || {
                git clone --branch 3.4.1 --depth 1 https://github.com/opencv/opencv.git
                pushd opencv
                git apply "${source_dir}"/scripts/raspberrypi3/opencv.patch
                git status
                popd
        }

        popd
}

yes_or_exit() {
        message=$1
        while true; do
                read -p "${message} [Y/n]" yn
                case $yn in
                        [Yy]* ) break;;
                        [Nn]* ) exit;;
                        * ) echo "Please answer yes or no.";;
                esac
        done
}

setup() {
        NUM_JOBS=$(nproc --all)
        no_apt="False"
        enable_server="False"

        while [[ $# -gt 0 ]]
        do
        key="$1"
        case $key in
                -y|--yes)
                answer_yes="True"
                shift # next argument
                ;;
                -h|--help)
                display_help="True"
                shift # next argument
                ;;
                -b|--builddir)
                build_dir=$2
                shift # past argument
                shift # past value
                ;;
                -d|--depsdir)
                deps_dir=$2
                shift # past argument
                shift # past value
                ;;
                -i|--depsinstalldir)
                deps_install_dir=$2
                shift # past argument
                shift # past value
                ;;
                -j|--jobs)
                NUM_JOBS=$2
                shift # past argument
                shift # past value
                ;;
                -bo|--buildopencv)
                build_opencv="True"
                shift # next argument
                ;;
                -op|--opencvpath)
                opencv_path=$2
                shift # next argument
                shift # next value
                ;;
                -ur|--udevrules)
                udev_rules="True"
                shift # next argument
                ;;
                -na|--no-apt)
                no_apt="True"
                shift # next argument
                ;;
                -es|--enable-server)
                enable_server="True"
                shift # next argument
                ;;
                *)    # unknown option
                POSITIONAL+=("$1") # save it in an array for later
                shift # past argument
                ;;
        esac
        done
        set -- "${POSITIONAL[@]}" # restore positional parameters

        if [[ "${display_help}" == "True" ]]; then
                print_help
                exit
        fi

        if [[ "${no_apt}" == "False" ]]; then
             install_required_packages
        fi

        if [[ -z "${build_dir}" ]]; then
                build_dir=$(pwd)/build
        fi

        echo "The sdk will be built in: ${build_dir}"

        if [[ -z "${deps_dir}" ]]; then
                deps_dir=$(pwd)/deps
        fi

        echo "The deps will be downloaded in: ${deps_dir}"

        if [[ -z "${deps_install_dir}" ]]; then
                deps_install_dir=${deps_dir}/installed
        fi

        echo "The deps will be installed in: ${deps_install_dir}"

        if [[ -z "${answer_yes}" ]]; then
             yes_or_exit "Do you want to continue?"
        fi

        mkdir -p "${build_dir}"
        mkdir -p "${deps_dir}"
        mkdir -p "${deps_install_dir}"

        pushd "${deps_install_dir}"
        deps_install_dir=$(pwd)
        popd

        get_deps_source_code ${deps_dir}

        if [[ "${build_opencv}" == "True" ]]; then
                get_opencv_source_code "${deps_dir}"
                OPENCV=3.4.1
        fi

        build_and_install_glog ${deps_dir}/glog ${deps_install_dir}/glog
        build_and_install_protobuf ${deps_dir}/protobuf ${deps_install_dir}/protobuf -DCMAKE_CXX_FLAGS=\"-latomic\"
        build_and_install_websockets ${deps_dir}/libwebsockets ${deps_install_dir}/websockets

        if [[ "${build_opencv}" == "True" ]]; then
                build_and_install_opencv ${deps_dir}/opencv ${deps_install_dir}/"opencv-${OPENCV}"
        fi

        CMAKE_OPTIONS="-DRASPBERRYPI=1 -DWITH_PYTHON=on -DWITH_OPENCV=on"
        PREFIX_PATH="${deps_install_dir}/glog;${deps_install_dir}/protobuf;${deps_install_dir}/websockets;"

        if [[ "${build_opencv}" == "True" ]]; then
                PREFIX_PATH="${PREFIX_PATH}${deps_install_dir}/opencv-${OPENCV};"
        else
             if [[ ! -z "${opencv_path}" ]]; then
                PREFIX_PATH="${PREFIX_PATH}${opencv_path};"
             fi
        fi

        pushd "${build_dir}"
        cmake "${source_dir}" ${CMAKE_OPTIONS} -DCMAKE_PREFIX_PATH="${PREFIX_PATH}"
        make -j ${NUM_JOBS}
		
        if [[ "${udev_rules}" == "True" ]]; then
             sudo make install-udev-rules
        fi

        popd

        if [[ "${enable_server}" == "True" ]]; then
             sudo cp ${source_dir}/utils/raspberrypi/tof-server.service /etc/systemd/system/
             sudo systemctl enable tof-server
        fi
}

setup $@
