cmake_minimum_required(VERSION 3.4)
project(cal_eeprom VERSION 0.0.1 LANGUAGES C CXX)

set (CMAKE_CXX_STANDARD 11)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_SOURCE_DIR}")

if (WIN32)
    set(PLATFORM_NAME "win")
	if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
		# using Visual Studio C++
		add_compile_definitions(_CRT_SECURE_NO_WARNINGS) #add preprocessor to disable warning for Microsoft deprecated function
	endif()
endif (WIN32)

# add the executable
add_executable(
	cal_eeprom
	main.cpp
	cal_eeprom.cpp)

	



