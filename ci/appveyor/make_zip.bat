mkdir C:\aditof-sdk\
pushd C:\aditof-sdk

if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2015" (
    set folder=vs14
) else if "%APPVEYOR_BUILD_WORKER_IMAGE%"=="Visual Studio 2017" (
    set folder=vs15
)
mkdir x64
mkdir x32
mkdir include
mkdir x64\%folder%
mkdir x32\%folder%
mkdir x64\%folder%\bin
mkdir x64\%folder%\lib
mkdir x32\%folder%\bin
mkdir x32\%folder%\lib

cp C:\projects\aditof-sdk\build\sdk\Release\aditof.dll C:\aditof-sdk\x64\%folder%\bin
cp C:\projects\aditof-sdk\build\sdk\Release\aditof.lib C:\aditof-sdk\x64\%folder%\lib

ren "C:\projects\aditof-sdk\build_debug\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\aditof-sdk\build_debug\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\aditof-sdk\build_debug\sdk\Debug\aditof.pdb" "aditofd.pdb"


cp C:\projects\aditof-sdk\build_debug\sdk\Debug\aditofd.dll C:\aditof-sdk\x64\%folder%\bin
cp C:\projects\aditof-sdk\build_debug\sdk\Debug\aditofd.pdb C:\aditof-sdk\x64\%folder%\bin
cp C:\projects\aditof-sdk\build_debug\sdk\Debug\aditofd.lib C:\aditof-sdk\x64\%folder%\lib

cp C:\projects\aditof-sdk\build_x32\sdk\Release\aditof.dll C:\aditof-sdk\x32\%folder%\bin
cp C:\projects\aditof-sdk\build_x32\sdk\Release\aditof.lib C:\aditof-sdk\x32\%folder%\lib

ren "C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditof.dll" "aditofd.dll"
ren "C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditof.lib" "aditofd.lib"
ren "C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditof.pdb" "aditofd.pdb"

cp C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditofd.dll C:\aditof-sdk\x32\%folder%\bin
cp C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditofd.pdb C:\aditof-sdk\x32\%folder%\bin
cp C:\projects\aditof-sdk\build_debug_x32\sdk\Debug\aditofd.lib C:\aditof-sdk\x32\%folder%\lib

cp -r C:\projects\aditof-sdk\sdk\include\ C:\aditof-sdk\include

7z a "C:\aditof-sdk-%folder%.zip" C:\aditof-sdk
appveyor PushArtifact C:\aditof-sdk-%folder%.zip

popd
