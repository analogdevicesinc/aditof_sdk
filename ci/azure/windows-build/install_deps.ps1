$ARCH=$Env:ARCH
$GENERATOR=$Env:COMPILER

cinst opencv --version 3.4.1
cinst openssl

$local_path=$pwd
mkdir deps_installed
mkdir -p deps_installed/glog
mkdir -p deps_installed/protobuf 
mkdir -p deps_installed/websockets
cp -r C:/'Program Files'/OpenSSL-Win64 deps_installed
cp -r C:/tools/opencv deps_installed

git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 
cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/glog" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4

cd $local_path
git clone --branch v3.2.3 --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_2_3
cd build_3_2_3
cmake -DOPENSSL_ROOT_DIR="$local_path/deps_installed/OpenSSL-Win64" -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/websockets" -G $GENERATOR ..
cmake --build . --target install --config Release -j 4 

cd $local_path
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 
cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX="$local_path/deps_installed/protobuf" -G $GENERATOR ../cmake
cmake --build . --target install --config Release -j 4