#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${ROS2_WS_DIR:-$SCRIPT_DIR}"

LIDAR_IP="192.168.11.1"
LIDAR_PORT=1445

# Function to check if Lidar responds to ping
check_lidar_ping() {
    ping -c 1 -W 1 "$LIDAR_IP" &> /dev/null
    return $?
}

# Function to check if Lidar TCP port is open
check_lidar_port() {
    timeout 3 bash -c "</dev/tcp/$LIDAR_IP/$LIDAR_PORT" 2>/dev/null
    return $?
}

# Wait for Lidar to respond to ping
while ! check_lidar_ping; do
    echo "Waiting for Lidar at $LIDAR_IP (ping)..."
    sleep 2
    # Optionally, add a timeout or log attempts
    # You can also check for a specific number of failures and reboot, etc.
done

echo "Lidar ping successful! Checking TCP port $LIDAR_PORT..."

# Wait for Lidar TCP port to be open
while ! check_lidar_port; do
    echo "Lidar port $LIDAR_PORT not ready, waiting 5 seconds..."
    sleep 5
    # Optionally, add a timeout or log attempts
    # You can also check for a specific number of failures and reboot, etc.
done

echo "Lidar is fully ready! Starting ROS2 nodes."

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Launch all nodes via single launch file
# (LiDAR driver + MQTT bridge + obstacle detection)
ros2 launch "$WORKSPACE_DIR/src/launch/robot_pi.launch.py"
