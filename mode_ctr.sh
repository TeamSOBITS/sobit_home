sobit_home_mode() {
    local domain_id=80
    export ROS_DOMAIN_ID=$domain_id
    export CYCLONEDDS_URI=${CYCLONEDDS_URI_PATH}
    echo "[SOBIT HOME] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}
default_mode() {
    export ROS_DOMAIN_ID=${HOST_ROS_DOMAIN_ID}
    echo "[Default] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}