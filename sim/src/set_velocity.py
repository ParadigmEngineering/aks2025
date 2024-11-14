#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class VelocityControl(Node):
    def __init__(self):
        super().__init__('velocity_control')
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

    def send_velocity(self, vx, vy, vz):
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = vy
        vel.linear.z = vz
        self.velocity_publisher.publish(vel)

def main():
    rclpy.init()
    node = VelocityControl()
    node.send_velocity(1.0, 0.0, 0.0)  # Example: Move at 1 m/s in the x-direction
    rclpy.spin(node)
    rclpy.shutdown()

