sobit_home_mode() {
    local domain_id=80
    ros2 daemon stop
    export ROS_DOMAIN_ID=$domain_id
    echo "[SOBIT HOME] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}
default_mode() {
    ros2 daemon stop
    export ROS_DOMAIN_ID=${HOST_ROS_DOMAIN_ID}
    echo "[Default] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}
default_mode
