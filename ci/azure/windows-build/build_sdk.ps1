cinst opencv --version 3.4.1
#cinst openssl
#cd C:\tools\opencv
#tree

mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="../deps_installed/glog;../deps_installed/protobuf;../deps_installed/websockets" -DOpenCV_DIR="C:\tools\opencv\build\x64\vc15\lib" ..
cmake --build . --target install --config Release -j 4
