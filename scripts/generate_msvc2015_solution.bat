:: Run this on windows to generate a MSVC 2015 solution targeted for x64

@echo off

pushd %~dp0%  
cd ..
set sdk_path=%CD%
popd

set build_path=%CD%\build

if not [%1]==[] set build_path=%1

if not exist %build_path% mkdir %build_path%
cd %build_path%

@echo Build files will be written to: %build_path%

cmake -G "Visual Studio 14 2015 Win64"^
 %sdk_path%
goto :eof
 
:usage
@echo Usage: .\%~nx0 ^<build directory(default="current directory")^>
exit /B 1