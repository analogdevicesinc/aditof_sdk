#cd deps_installed
#tree
#cd ..
#
#mkdir build
#cd build
#cmake -DCMAKE_PREFIX_PATH="../deps_installed/glog;../deps_installed/protobuf;../deps_installed/websockets;../deps_installed/opencv/build" -DOpenCV_DIR="../deps_installed/opencv/build/x64/vc15/lib" -DOpenCV_INCLUDE_DIR="../deps_installed/opencv/build/include" -DOPENSSL_INCLUDE_DIRS="../deps_installed/OpenSSL-Win64/include" ..
#cmake --build . --target install --config Release -j 4
