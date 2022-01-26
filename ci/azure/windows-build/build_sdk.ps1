#install OpenCV
cinst opencv --version 3.4.1

#build sdk
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="../deps_installed/glog;../deps_installed/protobuf;../deps_installed/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" -DOPENSSL_INCLUDE_DIRS="..\deps_installed\OpenSSL-Win64\include" ..
cmake --build . --target install --config Release -j 4
