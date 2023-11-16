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

helpFunction() {
  echo "Usage: ./ros2_setup -r <ros2 distro>"
  echo ""
  echo -e "  -r <ros2 distro>\tfoxy, galactic, humble"
  echo ""
  exit 1
}

ROS1_DISTRO=""
ROS2_DISTRO=""

while getopts "r:h" opt; do
  case "$opt" in
  r) ROS2_DISTRO="$OPTARG" ;;
  h) helpFunction ;;
  ?) helpFunction ;;
  esac
done

if [ "$ROS2_DISTRO" != "foxy" ] && [ "$ROS2_DISTRO" != "galactic" ] && [ "$ROS2_DISTRO" != "humble" ]; then
  echo -e "\033[1;31mInvalid ROS2 version.\033[0m"
  exit 1
fi

if [ "$ROS2_DISTRO" == "foxy" ] || [ "$ROS2_DISTRO" == "galactic" ]; then
  ROS1_DISTRO="noetic"
fi

# Update repository and install dependencies
echo -e "\033[1;31mUpdate repository and install dependencies.\033[0m"

sudo apt update
sudo apt upgrade -y
sudo apt install -y ssh net-tools terminator ntpdate curl vim git
sudo ntpdate ntp.ubuntu.com

# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install ROS2
echo -e "\033[1;31mInstall ROS2 $ROS2_DISTRO.\033[0m"

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-$ROS2_DISTRO-desktop-full python3-rosdep ros-$ROS2_DISTRO-rqt* ros-$ROS2_DISTRO-ros2-control ros-$ROS2_DISTRO-ros2-controllers ros-$ROS2_DISTRO-navigation2

source /opt/ros/$ROS2_DISTRO/setup.bash

sudo rosdep init
rosdep update

# Create the ROS2 workspace
echo -e "\033[1;31mCreate the ROS2 workspace.\033[0m"

if [ ! -d "$HOME/colcon_ws" ]; then
  echo -e "\033[1;31mCreating ROS2 workspace ...\033[0m"
  mkdir -p ~/colcon_ws/src
  cd ~/colcon_ws
  colcon build --symlink-install
fi

# Create the ROS enviornmnet file
echo -e "\033[1;31mCreate the ROS enviornmnet file.\033[0m"

if [ ! -f "$HOME/env.sh" ] && [ ! -f "/etc/ros/env.sh" ]; then
  echo -e "\nsource /etc/ros/env.sh" >>~/.bashrc
fi

if [ -f "$HOME/env.sh" ]; then
  sudo rm $HOME/env.sh
fi

if [ -f "/etc/ros/env.sh" ]; then
  sudo rm /etc/ros/env.sh
fi

if [ "$ROS2_DISTRO" == "foxy" ] || [ "$ROS2_DISTRO" == "galactic" ]; then
  echo -e "#!/bin/bash

# Please write the ROS environment variables here

export ROS_DISTRO=$ROS2_DISTRO

source /opt/ros/\$ROS_DISTRO/setup.bash

if [ \"\$ROS_DISTRO\" == \"$ROS1_DISTRO\" ]; then
  source ~/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
fi

if [ \"\$ROS_DISTRO\" == \"$ROS2_DISTRO\" ]; then
  alias cb='cd ~/colcon_ws && colcon build --symlink-install'

  source ~/colcon_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=101
fi" >>$HOME/env.sh
fi

if [ "$ROS2_DISTRO" == "humble" ]; then
  echo -e "#!/bin/bash

# Please write the ROS environment variables here

source /opt/ros/$ROS2_DISTRO/setup.bash
source ~/colcon_ws/install/setup.bash

alias cb='cd ~/colcon_ws && colcon build --symlink-install'
  
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=101" >>$HOME/env.sh
fi

sudo mv $HOME/env.sh /etc/ros/
