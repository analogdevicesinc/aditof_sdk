set configuration=%~1
set arch=%~2

echo "Installing websockets with config=%configuration% arch=%arch%"

if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2015" (
    set generator=Visual Studio 14 2015
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2017" (
    set generator=Visual Studio 15 2017
)

pushd deps

set folder_arch=%arch%

if "%arch%"=="Win64" (
    set arch= %arch%
    set arch_bits=64
) else (
    set arch_bits=32
)

if not exist "libwebsockets" ( git clone --branch v3.1-stable --depth 1  https://libwebsockets.org/repo/libwebsockets ) 
pushd libwebsockets
set build_dir="build_3_1_stable%folder_arch%"
if not exist %build_dir% ( mkdir %build_dir% )
pushd %build_dir%
cmake  -DOPENSSL_ROOT_DIR="C:\OpenSSL-Win" -DCMAKE_INSTALL_PREFIX="./local_path/websockets" -G "%generator%%arch%" ..
cmake --build . --target install --config %configuration%
popd
popd
popd
