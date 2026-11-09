#!/usr/bin/env python3
"""
Safety Zone + Heartbeat Node (ROS 2 Jazzy)

What this node does:
 - Subscribes:
     * /scan (LaserScan) for obstacle detection
     * /cmd_vel (Twist) to gate motion when safety stop is active
     * /robot/safety_radius (Float32) to update radius at runtime from MQTT bridge
 - Publishes:
     * robot_controller/cmd_vel_unstamped (Twist) safe output velocity
     * /safety_zone_marker (MarkerArray) RViz safety circle
     * /safety_zone_alert (String JSON) stop/clear alerts
     * /robot/heartbeat (String JSON) system status (lidar/mcu/radius)

Safety behavior:
 - If any valid LiDAR range <= SAFETY_RADIUS:
     * publishes stop alert JSON: {"msg":"stop","distance":...}
     * sends "stop" to Arduino
     * forces zero velocity output
 - When clear:
     * publishes clear alert JSON: {"msg":"clear","distance":null}
     * sends "clear" to Arduino
     * resumes immediately when RESUME_DELAY <= 0 (or delayed otherwise)

Arduino behavior:
 - Startup: if USE_ARDUINO=True, node blocks until Arduino connects (timeout + retry)
     and aborts startup if not found.
 - Runtime: checks connection periodically; on unplug => connected=false,
     on replug => auto reconnect and connected=true again.

Heartbeat behavior:
 - lidar.connected: TCP reachability to 192.168.11.1:1445
 - lidar.publishing: recent /scan activity within LIDAR_PUBLISH_TIMEOUT
 - mcu.connected: live serial connection state
 - safety_radius and obstacle_detected included for real-time monitoring
"""

import math
import json
import os
import sys
import time
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Float32

# ----------------- USER CONFIG -----------------
USE_ROS_BRIDGE = True          # Publish compact JSON on /safety_zone_alert
USE_ARDUINO = True             # Require Arduino presence and use it
# ARDUINO_SERIAL_PORT is still supported as env fallback but preferred to pass as ROS param
ARDUINO_BAUD = int(os.environ.get('ARDUINO_BAUD', '115200'))
# how long to keep retrying to open Arduino on startup (seconds). If exceeded, abort.
ARDUINO_CONNECT_TIMEOUT = float(os.environ.get('ARDUINO_CONNECT_TIMEOUT', '30.0'))
# how long between attempts while connecting (seconds)
ARDUINO_RETRY_INTERVAL = float(os.environ.get('ARDUINO_RETRY_INTERVAL', '2.0'))
# ------------------------------------------------

# Safety configuration
SAFETY_RADIUS     = 1.0    # meters  (can be changed at runtime via MQTT topic robot/safety_radius)
RESUME_DELAY      = 0.0    # seconds; if <= 0 -> immediate resume on clear

# Visualization config
MARKER_FRAME      = "base_link"
CIRCLE_RESOLUTION = 72
BORDER_WIDTH      = 0.06
FILL_ALPHA        = 0.12

COLOUR_CLEAR     = (1.0, 1.0, 0.35)
COLOUR_OBSTACLE  = (1.0, 0.25, 0.25)

# Topics
ALERT_TOPIC_ROS = "/safety_zone_alert"   # std_msgs/String (compact JSON)
HEARTBEAT_TOPIC = "/robot/heartbeat"     # std_msgs/String (JSON status)
HEARTBEAT_INTERVAL = float(os.environ.get('HEARTBEAT_INTERVAL', '0.5'))
LIDAR_PUBLISH_TIMEOUT = float(os.environ.get('LIDAR_PUBLISH_TIMEOUT', '3.0'))
# ------------------------------------------------

# optional: lazy import serial (pyserial). If missing, we disable Arduino usage.
try:
    import serial  # type: ignore
except Exception:
    serial = None


class SafetyZoneNode(Node):

    def __init__(self):
        super().__init__("safety_zone_node")

        # Declare ros parameter for arduino port so user can change in launch or at runtime.
        default_port = os.environ.get('ARDUINO_SERIAL_PORT', '/dev/my_arduino')
        self.declare_parameter('arduino_port', default_port)
        # Note: the node will read this parameter during startup attempts; changing it
        # at runtime won't be used unless you restart the node (because we block on startup).

        # State
        self.obstacle_in_zone = False
        self.robot_is_stopped = False
        self.last_cmd = Twist()
        self.is_moving = False
        self.clear_since = None

        # LiDAR tracking
        self.lidar_last_stamp = None   # time.time() of last /scan received
        self.lidar_scan_count = 0      # total scans received
        self.lidar_connected = False   # True if device reachable at 192.168.11.1:1445

        # QoS for lidar (BEST_EFFORT common for scans)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.create_subscription(LaserScan, "/scan", self.scan_callback, lidar_qos)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float32, "/robot/safety_radius", self.radius_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "robot_controller/cmd_vel_unstamped", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/safety_zone_marker", 10)

        # ROS-bridge alert publisher (std_msgs/String)
        if USE_ROS_BRIDGE:
            self.alert_pub = self.create_publisher(String, ALERT_TOPIC_ROS, 10)
            self.heartbeat_pub = self.create_publisher(String, HEARTBEAT_TOPIC, 10)
        else:
            self.alert_pub = None
            self.heartbeat_pub = None

        # Arduino serial state
        self.arduino_ok = False
        self.ser = None

        # Try to open Arduino (blocking with retries). If USE_ARDUINO==False we skip.
        if USE_ARDUINO:
            self.get_logger().info(
                f"Attempting to connect to Arduino (timeout={ARDUINO_CONNECT_TIMEOUT}s, "
                f"interval={ARDUINO_RETRY_INTERVAL}s)..."
            )
            connected = self._init_arduino_blocking(timeout=ARDUINO_CONNECT_TIMEOUT,
                                                    retry_interval=ARDUINO_RETRY_INTERVAL)
            if not connected:
                # Fail hard: we log and abort the process (user requested that node not run).
                self.get_logger().error(
                    f"Arduino did not connect within {ARDUINO_CONNECT_TIMEOUT}s. Aborting startup."
                )
                # allow logs to flush; then cleanly shutdown rclpy and exit non-zero.
                try:
                    # publish nothing; ensure we attempt a graceful shutdown
                    rclpy.shutdown()
                except Exception:
                    pass
                # Exit with non-zero code
                sys.exit(1)
        else:
            if serial is None:
                self.get_logger().info("USE_ARDUINO=False and pyserial not installed — continuing without Arduino.")
            else:
                self.get_logger().info("USE_ARDUINO=False — continuing without Arduino.")

        # Visualisation timer
        self.create_timer(0.1, self._publish_marker)

        # Heartbeat timer
        if USE_ROS_BRIDGE:
            self.create_timer(HEARTBEAT_INTERVAL, self._publish_heartbeat)
            self.create_timer(5.0, self._check_lidar_connection)  # check network every 5s

        # Arduino runtime health check (updates heartbeat quickly after unplug)
        if USE_ARDUINO:
            self.create_timer(1.0, self._check_arduino_connection)

        self.get_logger().info(
            f"SafetyZoneNode ready | radius={SAFETY_RADIUS} m | resume_delay={RESUME_DELAY} s | "
            f"ROS_BRIDGE={USE_ROS_BRIDGE} USE_ARDUINO={USE_ARDUINO} arduino_port={self._get_arduino_port()}"
        )

    # ----------------- Arduino helpers -----------------
    def _get_arduino_port(self) -> str:
        # read param value (declared in __init__)
        try:
            param = self.get_parameter('arduino_port')
            # parameter_value has .get_parameter_value in earlier rclpy versions; safer to inspect
            return param.get_parameter_value().string_value
        except Exception:
            # fallback to env / default
            return os.environ.get('ARDUINO_SERIAL_PORT', '/dev/my_arduino')

    def _init_arduino_blocking(self, timeout: float = 30.0, retry_interval: float = 2.0) -> bool:
        """
        Blocking attempt to open Arduino serial port.
        Retries until timeout seconds elapse. Returns True on success, False on final failure.
        """
        if serial is None:
            self.get_logger().error("pyserial not available — cannot use Arduino. Install pyserial.")
            return False

        port = self._get_arduino_port()
        deadline = time.time() + float(timeout)
        last_exception = None

        while time.time() <= deadline:
            try:
                self.get_logger().info(f"Opening Arduino serial {port} @ {ARDUINO_BAUD} ...")
                self.ser = serial.Serial(port, ARDUINO_BAUD, timeout=1)
                # allow Arduino native USB to reset and start sketch
                time.sleep(2.0)
                self.arduino_ok = True
                self.get_logger().info("Arduino serial ready.")
                return True
            except Exception as e:
                last_exception = e
                self.arduino_ok = False
                remaining = max(0.0, deadline - time.time())
                self.get_logger().warn(
                    f"Could not open Arduino serial port ({port}): {e} — retrying for up to {remaining:.1f}s..."
                )
                time.sleep(retry_interval)

                # re-read the parameter in case it was changed before next attempt
                port = self._get_arduino_port()

        # final failure
        self.get_logger().error(f"Failed to open Arduino serial after {timeout}s. Last error: {last_exception}")
        return False

    def _send_to_arduino(self, cmd: str) -> None:
        """Send plain text command to Arduino ('stop' or 'clear'), newline-terminated.
        If Arduino becomes unavailable during runtime, we mark arduino_ok False and stop trying to write.
        """
        if not USE_ARDUINO:
            return
        if not getattr(self, "arduino_ok", False):
            # If for some reason serial went away at runtime, do not attempt to reopen (we aborted at startup).
            return

        try:
            if getattr(self, "ser", None) is None:
                # no serial handle — mark as not OK
                self.arduino_ok = False
                self.get_logger().warn("Arduino serial handle missing at write time; disabling Arduino writes.")
                return
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.get_logger().debug(f"Arduino <- {cmd}")
        except Exception as e:
            # mark serial as unavailable and log. Node will not attempt reconnection (per your request).
            self.arduino_ok = False
            try:
                if getattr(self, "ser", None) is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
            finally:
                self.get_logger().warn(f"Failed to write to Arduino serial at runtime: {e} — Arduino writes disabled.")

    def _try_open_arduino_once(self) -> bool:
        """Single non-blocking open attempt used by runtime health check."""
        if serial is None:
            self.arduino_ok = False
            return False
        try:
            port = self._get_arduino_port()
            self.ser = serial.Serial(port, ARDUINO_BAUD, timeout=1)
            time.sleep(0.2)
            self.arduino_ok = True
            self.get_logger().info(f"Arduino connected at runtime: {port}")
            return True
        except Exception:
            self.ser = None
            self.arduino_ok = False
            return False

    def _check_arduino_connection(self) -> None:
        """Runtime check: mark Arduino disconnected if cable/port is gone."""
        if not USE_ARDUINO:
            return

        # No active serial handle => try to reconnect when user plugs it back in
        if getattr(self, "ser", None) is None:
            self._try_open_arduino_once()
            return

        try:
            # If pyserial knows port is closed, treat as disconnected
            if not self.ser.is_open:
                if self.arduino_ok:
                    self.get_logger().warn("Arduino disconnected (serial port closed).")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.arduino_ok = False
                return

            # If device node disappears after unplug, detect it
            port_name = getattr(self.ser, "port", None)
            if port_name and not os.path.exists(port_name):
                if self.arduino_ok:
                    self.get_logger().warn(f"Arduino disconnected (device node missing: {port_name}).")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.arduino_ok = False
                return

            # Touch serial driver state; can raise if unplugged/hung
            _ = self.ser.in_waiting
            self.arduino_ok = True
        except Exception as e:
            if self.arduino_ok:
                self.get_logger().warn(f"Arduino disconnected at runtime: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.arduino_ok = False

    # ----------------- runtime config -----------------
    def radius_callback(self, msg: Float32) -> None:
        global SAFETY_RADIUS
        new_radius = msg.data
        if new_radius <= 0:
            self.get_logger().warn(f"Ignored invalid safety_radius value: {new_radius}")
            return
        old = SAFETY_RADIUS
        SAFETY_RADIUS = new_radius
        self.get_logger().info(f"Safety radius updated: {old} m → {SAFETY_RADIUS} m")

    # ----------------- cmd_vel relay / blocking -----------------
    def cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.is_moving = (
            msg.linear.x != 0.0 or
            msg.linear.y != 0.0 or
            msg.linear.z != 0.0 or
            msg.angular.x != 0.0 or
            msg.angular.y != 0.0 or
            msg.angular.z != 0.0
        )

        if self.robot_is_stopped:
            self.cmd_pub.publish(Twist())
        else:
            self.cmd_pub.publish(msg)

    # ----------------- scan logic / detection -----------------
    def scan_callback(self, msg: LaserScan) -> None:
        self.lidar_last_stamp = time.time()
        self.lidar_scan_count += 1

        valid = [r for r in msg.ranges if math.isfinite(r) and (r > msg.range_min)]
        obstacle_now = any(r <= SAFETY_RADIUS for r in valid)

        min_distance = None
        if valid:
            nearest = min(valid)
            if nearest <= SAFETY_RADIUS:
                min_distance = float(round(nearest, 3))

        # OBSTACLE ENTERS
        if obstacle_now:
            # Clear any pending clear timer/state
            self.clear_since = None

            if not self.obstacle_in_zone:
                self.obstacle_in_zone = True
                self.get_logger().warn("Obstacle inside safety zone!")

                payload_obj = {"msg": "stop", "distance": min_distance if min_distance is not None else None}
                payload = json.dumps(payload_obj, separators=(",", ":"))

                if USE_ROS_BRIDGE and self.alert_pub is not None:
                    try:
                        self.alert_pub.publish(String(data=payload))
                    except Exception as e:
                        self.get_logger().warn(f"Failed to publish ROS alert: {e}")

                self._send_to_arduino("stop")

            if self.is_moving and not self.robot_is_stopped:
                self.robot_is_stopped = True
                self.cmd_pub.publish(Twist())
                self.get_logger().warn("Safety hold engaged — robot stopped.")

        # ZONE CLEAR
        else:
            # If we previously had an obstacle, handle transition
            if self.obstacle_in_zone:
                # Immediate resume behavior when RESUME_DELAY <= 0
                if RESUME_DELAY <= 0.0:
                    # publish a clear alert and resume immediately
                    self.obstacle_in_zone = False

                    payload = json.dumps({"msg": "clear", "distance": None}, separators=(",", ":"))
                    if USE_ROS_BRIDGE and self.alert_pub is not None:
                        try:
                            self.alert_pub.publish(String(data=payload))
                        except Exception as e:
                            self.get_logger().warn(f"Failed to publish ROS clear alert: {e}")

                    self._send_to_arduino("clear")
                    self.clear_since = None
                    self._resume_robot()
                    self.get_logger().info("Zone clear — resumed immediately (RESUME_DELAY <= 0).")
                else:
                    # Delayed resume behavior
                    if self.clear_since is None:
                        self.clear_since = self.get_clock().now()
                        self.get_logger().info(f"Zone clear — waiting {RESUME_DELAY:.0f}s before resuming…")

                        payload = json.dumps({"msg": "clear", "distance": None}, separators=(",", ":"))
                        if USE_ROS_BRIDGE and self.alert_pub is not None:
                            try:
                                self.alert_pub.publish(String(data=payload))
                            except Exception as e:
                                self.get_logger().warn(f"Failed to publish ROS clear alert: {e}")

                        self._send_to_arduino("clear")

                    elapsed = (self.get_clock().now() - self.clear_since).nanoseconds * 1e-9
                    if elapsed >= RESUME_DELAY:
                        self.obstacle_in_zone = False
                        self._resume_robot()

    # ----------------- resume behaviour -----------------
    def _resume_robot(self) -> None:
        if not self.robot_is_stopped:
            return
        self.robot_is_stopped = False
        self.get_logger().info("Safety hold released — robot may move again.")

    # ----------------- heartbeat publishing -----------------
    def _check_lidar_connection(self) -> None:
        """TCP probe to 192.168.11.1:1445 — sets lidar_connected."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            result = s.connect_ex(("192.168.11.1", 1445))
            s.close()
            self.lidar_connected = (result == 0)
        except Exception:
            self.lidar_connected = False

    def _publish_heartbeat(self) -> None:
        if self.heartbeat_pub is None:
            return

        now = time.time()
        lidar_publishing = (self.lidar_last_stamp is not None
                            and (now - self.lidar_last_stamp) < LIDAR_PUBLISH_TIMEOUT)

        status = {
            "lidar": {
                "connected": self.lidar_connected,
                "publishing": lidar_publishing,
                "scans_received": self.lidar_scan_count
            },
            "mcu": {
                "connected": self.arduino_ok
            },
            "safety_radius": SAFETY_RADIUS,
            "obstacle_detected": self.obstacle_in_zone
        }

        payload = json.dumps(status, indent=2, ensure_ascii=False)
        self.heartbeat_pub.publish(String(data=payload))

    # ----------------- marker publishing -----------------
    def _publish_marker(self) -> None:
        r, g, b = COLOUR_OBSTACLE if self.obstacle_in_zone else COLOUR_CLEAR
        markers = MarkerArray()

        zero_time = Time(seconds=0).to_msg()

        border = Marker()
        border.header.frame_id = MARKER_FRAME
        border.header.stamp = zero_time
        border.ns = "safety_zone"
        border.id = 0
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.scale.x = BORDER_WIDTH
        border.color.r = r
        border.color.g = g
        border.color.b = b
        border.color.a = 1.0

        for i in range(CIRCLE_RESOLUTION + 1):
            angle = 2.0 * math.pi * i / CIRCLE_RESOLUTION
            p = Point()
            p.x = SAFETY_RADIUS * math.cos(angle)
            p.y = SAFETY_RADIUS * math.sin(angle)
            p.z = 0.05
            border.points.append(p)
        markers.markers.append(border)

        fill = Marker()
        fill.header.frame_id = MARKER_FRAME
        fill.header.stamp = zero_time
        fill.ns = "safety_zone"
        fill.id = 1
        fill.type = Marker.CYLINDER
        fill.action = Marker.ADD
        fill.pose.position.x = 0.0
        fill.pose.position.y = 0.0
        fill.pose.position.z = 0.0
        fill.pose.orientation.w = 1.0
        fill.scale.x = SAFETY_RADIUS * 2.0
        fill.scale.y = SAFETY_RADIUS * 2.0
        fill.scale.z = 0.01
        fill.color.r = r
        fill.color.g = g
        fill.color.b = b
        fill.color.a = FILL_ALPHA
        markers.markers.append(fill)

        self.marker_pub.publish(markers)

    # ----------------- cleanup -----------------
    def destroy_node(self):
        if getattr(self, "arduino_ok", False) and getattr(self, "ser", None) is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyZoneNode()
        rclpy.spin(node)
    except SystemExit as e:
        # Raised by our startup if Arduino failed to connect; ensure clean shutdown
        try:
            if node is not None:
                node.get_logger().error("Exiting due to startup failure (Arduino not connected).")
        except Exception:
            pass
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        # propagate exit code
        raise
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.cmd_pub.publish(Twist())
                except Exception:
                    pass
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
