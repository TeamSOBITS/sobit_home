#!/bin/bash

echo "╔══╣ Setup: SOBIT HOME (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages
ros_packages=(
    "urg_node"
    "sobits_interfaces"
    "orbbecsdk_ros2"
    "dynamixel_hardware"
    "rm_motors_ros"
    "swerve_steering_controller"
    "ros2_laser_scan_merger"
    "tmc_wrs_gz"
    "aws_small_house_world"
    "sobits_gazebo_worlds"
    # "iai_hardware"
)

#Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone --recurse-submodules -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Go back to previous directory
cd ${DIR}

# Download required dependencies
python3 -m pip install --break-system-packages \
    transforms3d

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-limits \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-launch \
    ros-$ROS_DISTRO-launch-ros \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-actuator-msgs \
    ros-$ROS_DISTRO-gps-msgs \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces

# Set up the environment
sudo usermod -aG dialout $USERNAME

# Install Gazebo Harmonic with binaries
sudo apt-get update
sudo apt-get install -y \
    curl \
    lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y \
    gz-harmonic

# Create new rules (check if they already exist)
if [ ! -f /etc/udev/rules.d/99-dxl-sobit_home-lower.rules ] && [ ! -f /etc/udev/rules.d/99-dxl-sobit_home-upper.rules ]; then
    echo "Creating udev rules for SOBIT HOME Dynamixels..."
    # (1) Lower Dynamixel USB2DYNAMIXEL (mobile base)
    sudo bash -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"FT7W9E57\", SYMLINK+=\"ttyUSB-DXL-sobit_home-lower\", MODE=\"0666\"" > /etc/udev/rules.d/99-dxl-sobit_home-lower.rules'
    # (2) Upper Dynamixel USB2DYNAMIXEL (head, arm, hand)
    sudo bash -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"FT7W9E4N\", SYMLINK+=\"ttyUSB-DXL-sobit_home-upper\", MODE=\"0666\"" > /etc/udev/rules.d/99-dxl-sobit_home-upper.rules'

    # Apply udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi


# Reboot notice
echo "============================================================"
echo "==  A system reboot is recommended to apply all changes.  =="
echo "============================================================"


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBIT HOME (FINISHED) ╠══╝"
