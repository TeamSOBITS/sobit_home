# DDS mode switching for SOBIT HOME. Sourced from .bashrc; every new terminal starts in dds_local_mode.
#
#   dds_local_mode : DDS stays on this PC (loopback). Simulator, or the robot running standalone.
#   dds_lan_mode   : DDS between the NUC and the dev PC over the wired robot LAN, ROS_DOMAIN_ID=80.
#
# Use the same mode in every terminal of a machine: the two modes do not see each other.

# The profiles live next to this script, so a renamed or moved checkout needs no .bashrc edit.
_SOBIT_HOME_DDS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Subnet of the wired robot LAN (NUC 172.16.10.80, dev PC 172.16.10.70); cyclonedds_lan.xml pins it too.
SOBIT_HOME_LAN_PREFIX="172.16.10."

# Print the name of the wired interface on the robot LAN, or fail if there is none.
_sobit_home_wired_iface() {
    local iface
    for iface in $(ip -4 -o addr show | awk -v p="${SOBIT_HOME_LAN_PREFIX}" 'index($4, p) == 1 {print $2}'); do
        if [ ! -d "/sys/class/net/${iface}/wireless" ]; then
            echo "${iface}"
            return 0
        fi
    done
    return 1
}

# LAN: share DDS with the other machine on the wired robot LAN (cyclonedds_lan.xml).
dds_lan_mode() {
    local iface
    # Robot DDS traffic (cameras, point clouds) must never go over Wi-Fi, so stay local without a wired link.
    if ! iface=$(_sobit_home_wired_iface); then
        echo "[DDS local] No wired ${SOBIT_HOME_LAN_PREFIX}x interface found, staying in local mode."
        return 1
    fi
    export CYCLONEDDS_URI=file://${_SOBIT_HOME_DDS_DIR}/cyclonedds_lan.xml
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    ros2 daemon stop > /dev/null
    export ROS_DOMAIN_ID=80
    echo "[DDS LAN] NUC <-> dev PC over ${iface}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

# Local: keep DDS on loopback so nothing reaches the LAN or Wi-Fi (cyclonedds_local.xml).
dds_local_mode() {
    export CYCLONEDDS_URI=file://${_SOBIT_HOME_DDS_DIR}/cyclonedds_local.xml
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 daemon stop > /dev/null
    export ROS_DOMAIN_ID=${HOST_ROS_DOMAIN_ID}
    echo "[DDS local] this PC only, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

# Former names, kept so existing habits and scripts keep working.
sobit_home_mode() { dds_lan_mode "$@"; }
default_mode() { dds_local_mode "$@"; }

dds_local_mode
