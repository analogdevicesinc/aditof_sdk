set configuration=%~1
set arch=%~2

if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2015" (
    set generator=Visual Studio 14 2015
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2017" (
    set generator=Visual Studio 15 2017
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2019" (
    set generator=Visual Studio 16 2019
)

echo "%generator%"
echo "%arch%"
pushd deps

set folder_arch=%arch%

echo "%generator%%arch%"

if not exist "protobuf" ( git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf ) 
pushd protobuf
if not exist "build_3_9_0%folder_arch%" ( mkdir build_3_9_0%folder_arch% )
pushd build_3_9_0%folder_arch%
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=./local_path/protobuf -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -G "%generator%" -A "%arch%" ..\cmake\
cmake --build . --target install --config %configuration%
popd
popd
popd
