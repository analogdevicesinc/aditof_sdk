#install OpenCV
choco install opencv --version 3.4.1

#build sdk
mkdir build_Release
mkdir build_Debug

cd build_Release
cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Release/glog;../deps_installed/Release/protobuf;../deps_installed/Release/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" ..
cmake --build . --target install --config Release -j 4

cd ../build_Debug
cmake -DWITH_OPENCV=on -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="../deps_installed/Debug/glog;../deps_installed/Debug/protobuf;../deps_installed/Debug/websockets;..\deps_installed\OpenSSL-Win64" -DOpenCV_DIR="C:/tools/opencv/build/x64/vc15/lib" ..
cmake --build . --target install --config Debug -j 4

#generate developer kit binaries
cd ..
mkdir aditof_library
mkdir -p aditof_library\bin
mkdir -p aditof_library\lib
mkdir -p aditof_library\bin\python
cp -r sdk\include aditof_library

#copy dll
cp build_Release\sdk\Release\aditof.dll aditof_library\bin
cp build_Debug\sdk\Debug\aditof.dll aditof_library\bin\aditofd.dll

#copy lib
cp build_Release\sdk\Release\aditof.lib aditof_library\lib
cp build_Debug\sdk\Debug\aditof.lib aditof_library\lib\aditofd.lib
cp build_Debug\sdk\Debug\aditof.pdb aditof_library\lib\aditofd.pdb

#copy python, openSSL, openCV
cp build_Release\bindings\python\Release\aditofpython* aditof_library\bin\python
Compress-Archive -Path aditof_library -DestinationPath aditof_x64_vs16.zip

#generate Installer
SET PATH=packages\Tools.InnoSetup.5.6.1\tools
iscc build_Release\aditof-setup.iss

#publish artifacts
Get-ChildItem $env:BUILD_ARTIFACTSTAGINGDIRECTORY -Force -Recurse | Remove-Item -Force -Recurse
cp C:/aditof-setup.exe $env:BUILD_ARTIFACTSTAGINGDIRECTORY
cp aditof_x64_vs16.zip $env:BUILD_ARTIFACTSTAGINGDIRECTORY
