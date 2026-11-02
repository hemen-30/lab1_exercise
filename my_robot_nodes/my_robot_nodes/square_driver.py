#!/usr/bin/env python3
"""Drive the robot around a square with the /drive_distance and /rotate actions."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)
        self.have_odom = False
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def on_odom(self, msg):
        self.have_odom = True

    def wait_until_ready(self):
        self.get_logger().info("Waiting for the action servers and odometry")
        while rclpy.ok() and not self.drive_client.wait_for_server(timeout_sec=1.0):
            pass
        while rclpy.ok() and not self.rotate_client.wait_for_server(timeout_sec=1.0):
            pass
        while rclpy.ok() and not self.have_odom:
            rclpy.spin_once(self, timeout_sec=0.5)
        return rclpy.ok()

    def send(self, client, goal):
        """Send one goal, wait for its result, and say whether it succeeded."""
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def run(self):
        if not self.wait_until_ready():
            return
        side_length = float(self.get_parameter("side_length").value)
        self.get_logger().info(f"Driving a square with sides of {side_length:.2f} m")
        for side in range(1, 5):
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length
            if not self.send(self.drive_client, drive_goal):
                self.get_logger().error(f"Driving failed on side {side}, stopping")
                return

            rotate_goal = Rotate.Goal()
            rotate_goal.angle = math.pi / 2.0
            if not self.send(self.rotate_client, rotate_goal):
                self.get_logger().error(f"Turning failed after side {side}, stopping")
                return
            self.get_logger().info(f"Side {side} done")
        self.get_logger().info("Square finished")


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
