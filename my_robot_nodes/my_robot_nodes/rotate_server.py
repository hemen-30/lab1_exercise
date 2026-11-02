#!/usr/bin/env python3
"""Action server that turns the robot on the spot through a given angle,
measured with the yaw angle from /odom."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_difference(a, b):
    """a - b, wrapped into the range -pi to pi."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)
        self.yaw = None

        # One reentrant group, so the odometry callback keeps running
        # while a goal is being executed
        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            Odometry, "odom", self.on_odom, 10, callback_group=group
        )
        self.server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=group,
        )

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal_request):
        if goal_request.angle == 0.0:
            self.get_logger().info("Goal rejected: angle is zero")
            return GoalResponse.REJECT
        if self.yaw is None:
            self.get_logger().info("Goal rejected: no odometry yet")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def stop(self):
        self.cmd_vel.publish(Twist())

    def execute(self, goal_handle):
        goal = goal_handle.request.angle
        direction = 1.0 if goal > 0.0 else -1.0
        speed = self.get_parameter("turn_speed").value
        turned = 0.0
        last_yaw = self.yaw
        feedback = Rotate.Feedback()
        result = Rotate.Result()

        while abs(turned) < abs(goal) and rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                result.turned = float(turned)
                return result

            twist = Twist()
            twist.angular.z = direction * speed
            self.cmd_vel.publish(twist)
            time.sleep(0.05)

            # Add up the small change in yaw since the last reading
            yaw = self.yaw
            turned += angle_difference(yaw, last_yaw)
            last_yaw = yaw

            feedback.turned = float(turned)
            goal_handle.publish_feedback(feedback)

        self.stop()
        goal_handle.succeed()
        result.turned = float(turned)
        return result


def main():
    rclpy.init()
    node = RotateServer()
    # A goal is executed in a loop, so other callbacks need other threads
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
