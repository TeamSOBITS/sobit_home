#!/bin/bash

echo "╔══╣ Setup: SOBIT HOME (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages
ros_packages=(
    "urg_node"
    "sobits_interfaces"
    "sobits_robot_descriptor"
    "orbbecsdk_ros2"
    "realsense_ros"
    "dynamixel_hardware"
    "uirobot_hardware"
    "rm_motors_ros"
    # "swerve_steering_controller"
    "ros2_laser_scan_merger"
    "tmc_wrs_gz"
    "aws_small_house_world"
    "sobits_gazebo_worlds"
    "sobits_display"
    "usb_cam"
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

# Every ROS dependency is declared in the package.xml files and comes from
# rosdep, including those of the cloned sibling packages.
sudo apt-get update
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install -r -y -i --from-paths ${DIR}
for pkg in "${ros_packages[@]}"; do
    rosdep install -r -y -i --from-paths ${DIR}/../${pkg}
done

# Set up the environment
sudo usermod -aG dialout $USERNAME

# Install Gazebo Harmonic with binaries
sudo apt-get update
sudo apt-get install -y \
    curl \
    mpg321 \
    lsb-release gnupg \
    openssh-server \
    netcat-openbsd

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y \
    gz-harmonic

# MoveIt warehouse (stored planning scenes / robot states / constraints).
# The sqlite backend is installed by rosdep and is the default. MongoDB has no
# binary release for this ROS distribution and no rosdep rule, so it is opt-in
# and built from source together with its C++ driver:
#     WAREHOUSE_MONGO=1 ./install.sh
if [ "${WAREHOUSE_MONGO}" = "1" ]; then
    echo "╠══╣ Installing the MongoDB warehouse backend from source"
    sudo apt-get install -y \
        build-essential \
        cmake \
        libssl-dev \
        libsasl2-dev \
        mongodb-server-core || \
        echo "WARNING: mongodb-server-core unavailable; install mongod from the MongoDB apt repository"

    if [ ! -d ../warehouse_ros_mongo ]; then
        git clone -b ros2 https://github.com/moveit/warehouse_ros_mongo.git ../warehouse_ros_mongo
    fi
    echo "Cloned warehouse_ros_mongo. Build it with the rest of the workspace:"
    echo "    colcon build --packages-select warehouse_ros_mongo"
    echo "Then launch with warehouse_backend:=mongo"
fi

# Set up environment variables
echo "" >> $HOME/.bashrc
echo "# SOBIT HOME environment variables" >> $HOME/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc
echo "export HOST_ROS_DOMAIN_ID=\${ROS_DOMAIN_ID}" >> $HOME/.bashrc
echo "source ${DIR}/mode_ctr.sh" >> $HOME/.bashrc
echo "if [ \"\$ROS_DOMAIN_ID\" = \"80\" ]; then" >> $HOME/.bashrc
echo "    export DXL_X_LOWER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W9E4N-if00-port0\`" >> $HOME/.bashrc
echo "    export DXL_X_UPPER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W9E57-if00-port0\`" >> $HOME/.bashrc
echo "    export DXL_P_UPPER_PORT=\`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TCRFG-if00-port0\`" >> $HOME/.bashrc
echo "    export DXL_X_HAND_PORT=\`realpath /dev/serial/by-id/usb-BestTechnology_E160_E160-if00-port0\`" >> $HOME/.bashrc
echo "    export UM_PORT=\`realpath /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_dabce0b66407f0118d421f2e6d9880ab-if00-port0\`" >> $HOME/.bashrc
echo "    export HOME_CAM_LEFT_PORT=\"/dev/\$(ls /sys/bus/usb/devices/3-5.4:1.0/video4linux/ | sort -V | head -1)\"" >> $HOME/.bashrc
echo "    export HOME_CAM_RIGHT_PORT=\"/dev/\$(ls /sys/bus/usb/devices/3-6.4:1.0/video4linux/ | sort -V | head -1)\"" >> $HOME/.bashrc
echo "    export RM_CAN_PORT=\`for n in /sys/class/net/can*; do [ \"\$(cat \$n/device/../serial 2>/dev/null)\" = 005200624B45501420313352 ] && basename \$n; done\`" >> $HOME/.bashrc
echo "fi" >> $HOME/.bashrc
echo "" >> $HOME/.bashrc

echo "alias audio-start='bash \$HOME/colcon_ws/src/sobit_home/audio.sh start'" >> $HOME/.bashrc
echo "alias audio-stop='bash \$HOME/colcon_ws/src/sobit_home/audio.sh stop'" >> $HOME/.bashrc
echo "alias audio-status='bash \$HOME/colcon_ws/src/sobit_home/audio.sh status'" >> $HOME/.bashrc

source $HOME/.bashrc

# Reboot notice
echo "============================================================"
echo "==  A system reboot is recommended to apply all changes.  =="
echo "============================================================"


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBIT HOME (FINISHED) ╠══╝"