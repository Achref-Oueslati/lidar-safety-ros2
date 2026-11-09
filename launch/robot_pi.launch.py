#!/usr/bin/env python3
"""
robot_pi.launch.py - Main launch file for Raspberry Pi

Launches:
  1. Slamware LiDAR driver (scan-only mode)
  2. MQTT bridge (joystick control + safety alerts)
  3. Joystick MQTT to Twist converter
  4. Obstacle detection / safety zone node

Usage:
  ros2 launch src/launch/robot_pi.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


# Base path for config and scripts
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(BASE_DIR, 'config')
SCRIPTS_DIR = os.path.join(BASE_DIR, 'scripts')


def generate_launch_description():

    # =============================================
    # 1. Slamware LiDAR Driver (scan-only mode)
    # =============================================
    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='screen',
        parameters=[
            os.path.join(CONFIG_DIR, 'slamware_scan_only.yaml'),
        ]
    )

    # =============================================
    # 2. MQTT Bridge (joystick + safety alerts)
    # =============================================
    mqtt_bridge_node = Node(
        package='mqtt_client',
        executable='mqtt_client',
        name='mqtt_bridge',
        output='screen',
        parameters=[
            os.path.join(CONFIG_DIR, 'mqtt_bridge.yaml'),
        ]
    )

    # =============================================
    # 3. Obstacle Detection / Safety Zone
    # =============================================
    obstacle_node = ExecuteProcess(
        cmd=['python3', os.path.join(SCRIPTS_DIR, 'obstacle_detection.py')],
        name='obstacle_detection',
        output='screen',
    )

    return LaunchDescription([
        slamware_node,
        mqtt_bridge_node,
        obstacle_node
    ])
