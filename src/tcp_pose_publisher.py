#!/usr/bin/env python3
"""
tcp_pose_publisher.py

Polls RobotState.getTcpPose via the jsonrpc_service (provided by aubo_client_node)
on a timer and republishes the result as geometry_msgs/PoseStamped.

Concepts demonstrated:
  - Service client (async, with done callback)
  - Timer callback
  - geometry_msgs/PoseStamped construction
  - RPY → quaternion conversion
  - ReentrantCallbackGroup + MultiThreadedExecutor for non-blocking calls
  - ROS2 parameters

Usage:
  ros2 run aubo_ros2_projects tcp_pose_publisher.py
  (jsonrpc_service must already be running — see aubo_client.launch.py)

  Or use the bundled launch file:
  ros2 launch aubo_ros2_projects tcp_pose_publisher.launch.py
"""

import json
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from aubo_msgs.srv import JsonRpc
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert fixed-axis XYZ Euler angles (radians) → quaternion (x, y, z, w).

    AUBO getTcpPose returns [x, y, z, rx, ry, rz] where rx/ry/rz are
    rotation angles about the fixed X/Y/Z axes of the base frame.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TcpPosePublisher(Node):
    """Publishes the robot TCP pose as PoseStamped by polling jsonrpc_service."""

    def __init__(self) -> None:
        super().__init__('tcp_pose_publisher')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate', 10.0)   # Hz
        self.declare_parameter('frame_id', 'rob1/base_link')
        self.declare_parameter('topic', 'tcp_pose')

        rate: float = self.get_parameter('publish_rate').value
        self.frame_id: str = self.get_parameter('frame_id').value
        topic: str = self.get_parameter('topic').value

        # ── Callback group ───────────────────────────────────────────────────
        # ReentrantCallbackGroup lets the done-callback run while the timer is
        # also spinning — necessary when using MultiThreadedExecutor.
        self._cbg = ReentrantCallbackGroup()

        # ── Service client ───────────────────────────────────────────────────
        self._client = self.create_client(
            JsonRpc,
            'jsonrpc_service',
            callback_group=self._cbg,
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(PoseStamped, topic, 10)

        # ── Timer ────────────────────────────────────────────────────────────
        # Guard flag: prevents queuing multiple in-flight requests if the
        # robot is slow to respond.
        self._request_pending: bool = False

        self._timer = self.create_timer(
            1.0 / rate,
            self._timer_cb,
            callback_group=self._cbg,
        )

        self.get_logger().info(
            f'TcpPosePublisher started — {rate} Hz on "~/{topic}" '
            f'[frame: {self.frame_id}]'
        )

    # ── Timer callback ───────────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        if self._request_pending:
            self.get_logger().debug('Previous request still in flight — skipping tick.')
            return

        if not self._client.service_is_ready():
            self.get_logger().warn(
                'jsonrpc_service not ready — is aubo_client_node running?',
                throttle_duration_sec=5.0,
            )
            return

        req = JsonRpc.Request()
        req.cls = 'RobotState'
        req.func = 'getTcpPose'
        req.params = '[]'

        self._request_pending = True
        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    # ── Service response callback ─────────────────────────────────────────────

    def _on_response(self, future) -> None:
        self._request_pending = False

        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')
            return

        # aubo_client_node puts 'None' (the string) in error when all is well
        if res.error not in ('None', ''):
            self.get_logger().warn(f'getTcpPose returned error: {res.error}')
            return

        try:
            pose_vals = json.loads(res.result)
        except Exception as exc:
            self.get_logger().error(f'JSON parse failed on result "{res.result}": {exc}')
            return

        if not isinstance(pose_vals, list) or len(pose_vals) < 6:
            self.get_logger().error(f'Unexpected getTcpPose format: {pose_vals}')
            return

        x, y, z, rx, ry, rz = (float(v) for v in pose_vals[:6])
        qx, qy, qz, qw = rpy_to_quaternion(rx, ry, rz)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self._pub.publish(msg)
        self.get_logger().debug(
            f'TCP pose → pos=({x:.4f}, {y:.4f}, {z:.4f})  '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpPosePublisher()

    # MultiThreadedExecutor is required so the done callback and the timer
    # can execute concurrently without deadlocking.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TcpPosePublisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
