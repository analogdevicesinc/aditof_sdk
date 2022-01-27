#install OpenCV
cinst opencv --version 3.4.1

#build sdk
mkdir build
cd build
cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/glog;../deps_installed/protobuf;../deps_installed/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" -DOPENSSL_INCLUDE_DIRS="..\deps_installed\OpenSSL-Win64\include" ..
cmake --build . --target install --config Release -j 4

#generate developer kit binaries
mkdir aditof_library
mkdir -p aditof_library\bin
mkdir -p aditof_library\lib
mkdir -p aditof_library\bin\python
cp -r ../sdk/include aditof_library
cp sdk\Release\aditof.dll aditof_library\bin
cp sdk\Release\aditof.lib aditof_library\lib
cp bindings\python\Release\aditofpython* aditof_library\bin\python
Compress-Archive -Path aditof_library -DestinationPath aditof_x64_vs16.zip

#generate Installer
SET PATH=packages\Tools.InnoSetup.5.6.1\tools
iscc ..\cmake\aditof-setup.iss.cmakein

#publish artifacts
Get-ChildItem $env:BUILD_ARTIFACTSTAGINGDIRECTORY -Force -Recurse | Remove-Item -Force -Recurse
cp C:/aditof-setup.exe $env:BUILD_ARTIFACTSTAGINGDIRECTORY
cp aditof_x64_vs16.zip $env:BUILD_ARTIFACTSTAGINGDIRECTORY