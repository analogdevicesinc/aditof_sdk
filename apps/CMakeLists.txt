cmake_minimum_required(VERSION 2.8)
project(apps)

if (DRAGONBOARD)
        add_subdirectory(server)
        add_subdirectory(uvc-app)
endif()

if (RASPBERRYPI)
        add_subdirectory(server)
endif()
