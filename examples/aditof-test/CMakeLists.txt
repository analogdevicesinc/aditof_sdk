cmake_minimum_required(VERSION 2.8)
project(aditof-test)

find_package(OpenCV)
if(OPENCV_FOUND OR
   OPENCV_CORE_FOUND) # we check for this because the OpenCVConfig.cmake(v2.4.9.1) used on dragonboards fails to set OPENCV_FOUND
	if(OpenCV_VERSION VERSION_LESS "3.0.0")
		message(STATUS "USING OPENCV Version 2")
		add_definitions(-DOPENCV2)
	endif(OpenCV_VERSION VERSION_LESS "3.0.0")
else()
	if (NOT OpenCV_INCLUDE_DIR OR NOT OpenCV_LIBS)
		message(FATAL_ERROR "find_package(openCV) failed. OpenCV_INCLUDE_DIR and OpenCV_LIBS need to be set.")
	else()
		message(STATUS "OpenCV_INCLUDE_DIR and OpenCV_LIBS have been set manually.")
	endif()
endif()

if(DATA_HANDLING)
	add_definitions(-DDATA_HANDLING)
endif()

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} PRIVATE aditof ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME} PRIVATE aditof)
