sobit_home_mode() {
    export CYCLONEDDS_URI=${REMOTE_CYCLONEDDS_URI}
    local domain_id=80
    ros2 daemon stop
    export ROS_DOMAIN_ID=$domain_id
    echo "[SOBIT HOME] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}
default_mode() {
    unset CYCLONEDDS_URI
    ros2 daemon stop
    export ROS_DOMAIN_ID=${HOST_ROS_DOMAIN_ID}
    echo "[Default] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}
default_mode
