cmake_minimum_required(VERSION 2.8)
PROJECT(wingetopt)

add_library(wingetopt STATIC src/getopt.c src/getopt.h)

target_include_directories(${PROJECT_NAME} PUBLIC src/)
