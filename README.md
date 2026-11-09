# LiDAR Safety ROS 2 Workspace

ROS 2 Jazzy workspace for a Raspberry Pi robot using a Slamware LiDAR, MQTT
telemetry, and an obstacle-safety node.

## Components

- `src/slamware_ros_sdk` and `src/slamware_sdk`: LiDAR driver and SDK
- `src/scripts/obstacle_detection.py`: scan-based safety stop and status reporting
- `src/config`: MQTT bridge and LiDAR driver configuration
- `src/launch/robot_pi.launch.py`: launches the driver, bridge, and safety node
- `launch_lidar_mqtt.sh`: waits for the LiDAR, then starts the launch file

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Set the broker host and replace the example MQTT username/password in
`src/config/mqtt_bridge.yaml`. Set the local broker address in
`src/config/mqtt_lidar.yaml`. Keep real credentials in an untracked local
configuration; never commit them.

For a connected robot, run:

```bash
./launch_lidar_mqtt.sh
```

The script assumes the LiDAR is reachable at its configured network address and
that ROS 2 Jazzy is installed. The workspace path can be overridden with
`ROS2_WS_DIR`.
