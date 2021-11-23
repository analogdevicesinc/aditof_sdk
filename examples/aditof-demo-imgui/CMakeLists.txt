cmake_minimum_required(VERSION 2.8)
project(aditof-demo-imgui)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE
		aditof
		imgui::imgui
		${OPENGL_gl_LIBRARY}
		glfw
		${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
			"${PROJECT_SOURCE_DIR}/../deps/imgui/src/examples"
			"${PROJECT_SOURCE_DIR}/../deps/imgui/src"
)
