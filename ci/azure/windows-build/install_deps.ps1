$ARCH=$Env:ARCH
$GENERATOR=$Env:COMPILER

$local_path=$pwd
mkdir deps_installed

mkdir -p deps_installed/Release/glog
mkdir -p deps_installed/Debug/glog

mkdir -p deps_installed/Release/protobuf
mkdir -p deps_installed/Debug/protobuf 

mkdir -p deps_installed/Release/websockets
mkdir -p deps_installed/Debug/websockets

#Install glog
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5_Release
mkdir build_0_3_5_Debug

cd build_0_3_5_Release
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/glog" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4

cd ../build_0_3_5_Debug
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/glog" -G $GENERATOR ..
cmake --build . --target install --config Debug -j 4

#Install websockets
cd $local_path
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3_Release
mkdir build_3_2_3_Debug

cd build_3_2_3_Release
cmake -DLWS_WITH_SSL=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/websockets" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4 

cd ../build_3_2_3_Debug
cmake -DLWS_WITH_SSL=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/websockets" -G $GENERATOR ..
cmake --build . --target install --config Debug -j 4

cd $local_path
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0_Release
mkdir build_3_9_0_Debug

cd build_3_9_0_Release
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Release/protobuf" -G $GENERATOR ../cmake
cmake --build . --target install --config Release -j 4

cd ../build_3_9_0_Debug
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/Debug/protobuf" -G $GENERATOR ../cmake
cmake --build . --target install --config Debug -j 4
