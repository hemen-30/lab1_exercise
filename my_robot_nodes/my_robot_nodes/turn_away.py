#!/usr/bin/env python3
"""Drive forward, and turn on the spot away from an obstacle that is too close.
Speeds are parameters, and the robot's state is published on /robot_status."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)

        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

    def on_obstacle(self, msg):
        # Read the parameters on every message, so a change takes effect at once
        forward_speed = self.get_parameter("forward_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        twist = Twist()
        status = RobotStatus()
        status.obstacle_distance = float(msg.distance)

        if not msg.too_close:
            twist.linear.x = forward_speed
            status.state = "forward"
        else:
            # angle > 0 means the obstacle is on the left, so turn right
            if msg.angle > 0.0:
                twist.angular.z = -turn_speed
            else:
                twist.angular.z = turn_speed
            status.state = "turning"

        self.cmd_vel.publish(twist)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
