set(UDEV_RULES_PATH "/etc/udev/rules.d" CACHE STRING "Target directory for udev rules installation.")
if(DRAGONBOARD)
    set(ADITOF_UDEV_RULES "${CMAKE_SOURCE_DIR}/utils/dragonboard/53-aditofsdkdragonboard.rules")  
elseif(RASPBERRYPI)
    set(ADITOF_UDEV_RULES "${CMAKE_SOURCE_DIR}/utils/raspberrypi/53-aditofsdkraspberrypi.rules")

elseif(JETSON)
    set(ADITOF_UDEV_RULES "${CMAKE_SOURCE_DIR}/utils/jetson/53-aditofsdkjetson.rules")
elseif(XAVIER)
    set(ADITOF_UDEV_RULES "${CMAKE_SOURCE_DIR}/utils/xavier/53-aditofsdkxavier.rules")
    
endif()
add_custom_target(install-udev-rules
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ADITOF_UDEV_RULES} ${UDEV_RULES_PATH}
    COMMAND udevadm control --reload-rules
    COMMAND udevadm trigger
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
