#!/bin/bash

DISTRO_CODENAME=$( (lsb_release -sc || . /etc/os-release; echo ${VERSION_CODENAME} ) 2>/dev/null)

#determine ROS distribution
if [ $DISTRO_CODENAME = "bionic" ]
then
	ROS_DISTRO="melodic"

elif [ $DISTRO_CODENAME = "focal" ]
then
        ROS_DISTRO="noetic"
        
elif [ $DISTRO_CODENAME = "buster" ]
then
        ROS_DISTRO="melodic"
fi


mkdir -p ${CATKIN_WS}/src
cd ${CATKIN_WS}/src
ln -sf ${ROS_SRC_DIR}/aditof_roscpp/ .

cd ${CATKIN_WS}
source /opt/ros/$ROS_DISTRO/setup.sh

#we have to save the old value of CMAKE_INSTALL_PREFIX in another variable
#because after running the setup.sh script, the variable will be reassigned
catkin_make -DADITOF_CMAKE_INSTALL_PREFIX=$1 -DADITOF_CMAKE_PREFIX_PATH=$2
