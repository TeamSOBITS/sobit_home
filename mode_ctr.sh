# DDS mode switching for SOBIT HOME.

# Remote: talk to the real robot at ROS_DOMAIN_ID=80 over the network.
sobit_home_mode() {
    export CYCLONEDDS_URI=${REMOTE_CYCLONEDDS_URI}
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    local domain_id=80
    ros2 daemon stop > /dev/null
    export ROS_DOMAIN_ID=$domain_id
    echo "[SOBIT HOME] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

# Default: keep discovery on loopback so DDS traffic never reaches the LAN.
default_mode() {
    export CYCLONEDDS_URI=${LOCAL_CYCLONEDDS_URI}
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 daemon stop > /dev/null
    export ROS_DOMAIN_ID=${HOST_ROS_DOMAIN_ID}
    echo "[Default] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

default_mode
