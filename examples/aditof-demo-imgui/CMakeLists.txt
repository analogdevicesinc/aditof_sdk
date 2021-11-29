cmake_minimum_required(VERSION 2.8)
project(aditof-demo-imgui)

find_package(OpenGL REQUIRED)

add_executable(${PROJECT_NAME}
			aditofdemorecorder.cpp
			aditofdemocontroller.cpp
			view.cpp
			main.cpp
)

if( WIN32 )
	target_link_libraries(${PROJECT_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/../deps/imgui/src/examples/libs/glfw/lib-vc2010-64/glfw3.lib")
elseif(APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE "${PROJECT_SOURCE_DIR}/../deps/imgui/lib/macOS/glfw/lib-x86_64/libglfw3.a")
elseif (UNIX)
	find_package(glfw3 REQUIRED)
	target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)

target_link_libraries(${PROJECT_NAME} PRIVATE
		aditof
		imgui::imgui
		${OPENGL_gl_LIBRARY}
		${CMAKE_DL_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
			"${PROJECT_SOURCE_DIR}/../deps/imgui/src/examples"
			"${PROJECT_SOURCE_DIR}/../deps/imgui/src"
)

# Post build actions
if( WIN32 )
    add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:aditof> $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
    )
endif()