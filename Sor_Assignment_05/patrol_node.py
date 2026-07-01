#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints


# --- Hardcode at least 3 distinct waypoints here: (x, y, theta_rad) ---
WAYPOINTS = [
    (0.893, 1.133, 0.766),
    (0.818, -0.945, -0.886),
    (-0.763, -0.099, 0.015),
]


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self._last_reported_index = -1
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', latched_qos
        )

        self.get_logger().info('Patrol node initialized. Waiting for Nav2 action server...')

    def set_initial_pose(self, x: float, y: float, theta: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f'Published initial pose: ({x}, {y}, {theta})')

    def build_pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_waypoints(self):
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available. Dispatching waypoints...')

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [
            self.build_pose_stamped(x, y, theta) for (x, y, theta) in WAYPOINTS
        ]

        self.get_logger().info('Navigating to Waypoint 1...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        current_index = feedback_msg.feedback.current_waypoint
        total = len(WAYPOINTS)

        if current_index > self._last_reported_index:
            if self._last_reported_index >= 0:
                self.get_logger().info(
                    f'Waypoint {self._last_reported_index + 1} Reached!'
                )
            if current_index < total:
                self.get_logger().info(
                    f'Navigating to Waypoint {current_index + 1}/{total}...'
                )
            self._last_reported_index = current_index

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected by the action server.')
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        total = len(WAYPOINTS)

        if not missed:
            self.get_logger().info(
                f'Waypoint {total} Reached! Patrol complete -- all {total} waypoints visited.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()

    node.send_waypoints()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()