#!/bin/bash

# Software License Agreement (BSD License)
#
# Authors : Brighten Lee <shlee@roas.co.kr>
#
# Copyright (c) 2021, ROAS Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.

helpFunction()
{
  echo "Usage: ./ros2_setup -r <ros2 distro>"
  echo ""
  echo -e "  -r <ros2 distro>\tfoxy"
  echo ""
  exit 1
}

ROS2_DISTRO=""

while getopts "r:h" opt
do
  case "$opt" in
    r) ROS2_DISTRO="$OPTARG" ;;
    h) helpFunction ;;
    ?) helpFunction ;;
  esac
done

if [ "$ROS2_DISTRO" != "foxy" ]; then 
  echo -e "\033[1;31mInvalid ROS2 version.\033[0m"
  exit 1
fi

if [ "$ROS2_DISTRO" == "foxy" ]; then 
  ROS1_DISTRO="noetic"
fi


# Update repository and install dependencies
echo -e "\033[1;31mStarting PC setup ...\033[0m"
sudo apt update
sudo apt upgrade -y
sudo apt install -y ssh net-tools terminator chrony ntpdate curl vim git setserial
sudo ntpdate ntp.ubuntu.com


# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8


# Add the ROS2 apt repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Install development tools and ROS2 tools
sudo apt install -y build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget
python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest


# Install ROS2
echo -e "\033[1;31mStarting ROS2 $ROS_DISTRO installation ...\033[0m"
sudo apt update && sudo apt install -y ros-foxy-desktop python3-rosdep ros-$ROS_DISTRO-rqt* ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-navigation2

source /opt/ros/$ROS_DISTRO/setup.bash

sudo rosdep init
rosdep update


# Create the ROS2 workspace
if [ ! -d "$HOME/colcon_ws" ]; then
  echo -e "\033[1;31mCreating ROS2 workspace ...\033[0m"
  mkdir -p ~/colcon_ws/src
  cd ~/colcon_ws
  colcon build --symlink-install
fi


# Create the ROS enviornmnet file
if [ ! -f "$HOME/env.sh" ] && [ ! -f "/etc/ros/env.sh" ]; then
  echo -e "\nsource /etc/ros/env.sh" >> ~/.bashrc
fi

if [ -f "$HOME/env.sh" ]; then
 sudo rm $HOME/env.sh
fi

if [ -f "/etc/ros/env.sh" ]; then
 sudo rm /etc/ros/env.sh
fi

echo -e "#!/bin/bash

# Please write the ROS environment variables here

export ROS_DISTRO=$ROS2_DISTRO

source /opt/ros/\$ROS_DISTRO/setup.bash

if [ \"\$ROS_DISTRO\" = \"$ROS1_DISTRO\" ]; then
  source ~/catkin_ws/devel/setup.bash
  alias cw='cd ~/catkin_ws'
  alias cs='cd ~/catkin_ws/src'
  alias cm='cd ~/catkin_ws && catkin_make'
  export ROS_MASTER_URI=http://localhost:11311
fi

if [ \"\$ROS_DISTRO\" = \"$ROS2_DISTRO\" ]; then
  source ~/colcon_ws/install/setup.bash
  alias cw='cd ~/colcon_ws'
  alias cs='cd ~/colcon_ws/src'
  alias cb='cd ~/colcon_ws && colcon build --symlink-install'
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=101
fi" >> $HOME/env.sh

sudo mv $HOME/env.sh /etc/ros/


# Install RealSense SDK
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install -y librealsense2-dkms librealsense2-utils
