#!/usr/bin/env python3
"""Drive forward, and turn on the spot away from an obstacle that is too close."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.msg import ObstacleInfo


class TurnAway(Node):

    def __init__(self):
        super().__init__("turn_away")
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

    def on_obstacle(self, msg):
        twist = Twist()
        if not msg.too_close:
            twist.linear.x = 0.2
        elif msg.angle > 0.0:
            # Obstacle on the left: turn right
            twist.angular.z = -0.5
        else:
            # Obstacle on the right or straight ahead: turn left
            twist.angular.z = 0.5
        self.cmd_vel.publish(twist)


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
