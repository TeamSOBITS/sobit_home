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
    "uirobot_hardware"
    "rm_motors_ros"
    # "swerve_steering_controller"
    "ros2_laser_scan_merger"
    "tmc_wrs_gz"
    "aws_small_house_world"
    "sobits_gazebo_worlds"
    "sobits_display"
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
    ros-$ROS_DISTRO-control-msgs \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-controller-manager-msgs \
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
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-ros-perception \
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
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Set up the environment
sudo usermod -aG dialout $USERNAME

# Install Gazebo Harmonic with binaries
sudo apt-get update
sudo apt-get install -y \
    curl \
    mpg321 \
    lsb-release gnupg \
    openssh-server

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y \
    gz-harmonic

# Set up environment variables
echo "" >> /home/$USERNAME/.bashrc
echo "# SOBIT HOME environment variables" >> /home/$USERNAME/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc
echo "export HOST_ROS_DOMAIN_ID=\${ROS_DOMAIN_ID}" >> /home/$USERNAME/.bashrc
echo "export REMOTE_CYCLONEDDS_URI=file://${DIR}/cyclonedds_profile.xml" >> /home/$USERNAME/.bashrc
echo "source ${DIR}/mode_ctr.sh" >> /home/$USERNAME/.bashrc
echo "if [ \"\$ROS_DOMAIN_ID\" = \"80\" ]; then" >> /home/$USERNAME/.bashrc
echo "    export DXL_X_LOWER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W9E4N-if00-port0\`" >> /home/$USERNAME/.bashrc
echo "    export DXL_X_UPPER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W9E57-if00-port0\`" >> /home/$USERNAME/.bashrc
echo "    export DXL_P_UPPER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TCRFG-if00-port0\`" >> /home/$USERNAME/.bashrc
echo "    export DXL_X_HAND_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7922NO-if00-port0\`" >> /home/$USERNAME/.bashrc
echo "    export UM_PORT=\`realpath /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_dabce0b66407f0118d421f2e6d9880ab-if00-port0\`" >> /home/$USERNAME/.bashrc
echo "    export HOME_CAM_LEFT_PORT=\"/dev/\$(ls /sys/bus/usb/devices/3-5.4:1.0/video4linux/ | sort -V | head -1)\"" >> /home/$USERNAME/.bashrc
echo "    export HOME_CAM_RIGHT_PORT=\"/dev/\$(ls /sys/bus/usb/devices/3-6.4:1.0/video4linux/ | sort -V | head -1)\"" >> /home/$USERNAME/.bashrc
echo "fi" >> /home/$USERNAME/.bashrc
echo "" >> /home/$USERNAME/.bashrc
source /home/$USERNAME/.bashrc

# Reboot notice
echo "============================================================"
echo "==  A system reboot is recommended to apply all changes.  =="
echo "============================================================"


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBIT HOME (FINISHED) ╠══╝"
