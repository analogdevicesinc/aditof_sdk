cinst InnoSetup
set PATH=%PATH%;"C:\Program Files (x86)\Inno Setup 5"
if not exist "C:\tools\opencv" ( cinst opencv )
cd %APPVEYOR_BUILD_FOLDER%
if not exist "deps" ( mkdir deps )
cd deps
if not exist "glog" ( git clone https://github.com/google/glog ) 
cd glog
git checkout tags/v0.3.5
if not exist "build_0_3_5" ( mkdir build_0_3_5 )
cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 15 2017 Win64" ..
cmake --build . --target install --config Release
