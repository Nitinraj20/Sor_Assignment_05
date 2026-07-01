#!/usr/bin/env python3

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

D0 = 0.075
L1 = 0.2
L23 = 0.425

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
]


def compute_ik(x, y, z, elbow_up=True):
    theta1 = math.atan2(y, x)

    r = math.hypot(x, y)
    z_prime = z - D0

    dist_sq = r * r + z_prime * z_prime
    max_reach = L1 + L23
    min_reach = abs(L1 - L23)
    dist = math.sqrt(dist_sq)

    if dist > max_reach or dist < min_reach:
        return None

    D = (dist_sq - L1 * L1 - L23 * L23) / (2 * L1 * L23)
    D = max(-1.0, min(1.0, D))

    theta3 = math.acos(D) if elbow_up else -math.acos(D)
    theta2 = math.atan2(r, z_prime) - math.atan2(
        L23 * math.sin(theta3),
        L1 + L23 * math.cos(theta3)
    )
    theta4 = 0.0

    return theta1, theta2, theta3, theta4


class ArmIKMover(Node):
    def __init__(self, target, duration_sec):
        super().__init__('arm_ik_mover')
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.target = target
        self.duration_sec = duration_sec

    def send_goal(self):
        angles = compute_ik(*self.target)
        if angles is None:
            self.get_logger().error(f'Target {self.target} is out of reach.')
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(
            f'IK solution for {self.target}: '
            f'theta1={math.degrees(angles[0]):.1f} deg, '
            f'theta2={math.degrees(angles[1]):.1f} deg, '
            f'theta3={math.degrees(angles[2]):.1f} deg, '
            f'theta4={math.degrees(angles[3]):.1f} deg'
        )

        point = JointTrajectoryPoint()
        point.positions = list(angles)
        point.time_from_start = Duration(sec=int(self.duration_sec), nanosec=0)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        goal_msg.trajectory.points = [point]

        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        self.get_logger().info('Sending goal...')
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: error_code={result.error_code}')
        rclpy.shutdown()


def main():
    if len(sys.argv) < 4:
        print('Usage: ros2 run <pkg> arm_ik_mover <x> <y> <z> [duration_sec]')
        sys.exit(1)

    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    duration_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0

    rclpy.init()
    node = ArmIKMover((x, y, z), duration_sec)
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()