cmake_minimum_required(VERSION 3.2)
project(aditof-daemon C)

add_executable(${PROJECT_NAME} main.c gpios.c leds.c)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "aditof-daemon")
