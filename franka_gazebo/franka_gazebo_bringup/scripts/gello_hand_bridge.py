#!/usr/bin/env python3
"""Relay the GELLO trigger opening to the simulated Franka Hand."""

import math

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GelloHandBridge(Node):
    """Translate normalized opening into a target position for each finger."""

    def __init__(self):
        super().__init__('gello_hand_bridge')
        self.target = None
        self.commands = self.create_publisher(
            JointTrajectory, '/gello_finger_controller/joint_trajectory', 1
        )
        self.create_subscription(
            Float32, '/gripper/gripper_client/target_gripper_width_percent', self.set_width, 1
        )
        # Repeat the last received target when the controller starts after the bridge.
        self.create_timer(0.5, self.publish_target)

    def set_width(self, message):
        """Apply significant changes in trigger opening immediately."""
        if not math.isfinite(message.data):
            return
        # The FR3 Hand opens 80 mm in total, or 40 mm per finger.
        position = 0.04 * min(1.0, max(0.0, message.data))
        if self.target is None or abs(self.target - position) >= 0.0001:
            self.target = position
            self.publish_target()

    def publish_target(self):
        """Send the latest target to the two-finger trajectory controller."""
        if self.target is None:
            return
        point = JointTrajectoryPoint(
            positions=[self.target, self.target],
            velocities=[0.0, 0.0],
            time_from_start=Duration(nanosec=200000000),
        )
        self.commands.publish(
            JointTrajectory(joint_names=['fr3_finger_joint1', 'fr3_finger_joint2'], points=[point])
        )


def main():
    """Run the bridge until the ROS context shuts down."""
    rclpy.init()
    bridge = GelloHandBridge()
    try:
        try:
            rclpy.spin(bridge)
        finally:
            bridge.destroy_node()
            rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
