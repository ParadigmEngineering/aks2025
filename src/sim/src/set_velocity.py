#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from rclpy.duration import Duration

class VelocityControl(Node):
    def __init__(self):
        super().__init__('velocity_control')
        self.get_logger().info("Initializing VelocityControl Node")
        
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for the SetMode service to be available
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetMode service to be available...')

    def send_velocity(self, vx, vy, vz):
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = vy
        vel.linear.z = vz
        self.velocity_publisher.publish(vel)
        self.get_logger().info(f"Published velocity: x={vx}, y={vy}, z={vz}")

def main():
    rclpy.init()
    node = VelocityControl()
    rate = node.create_rate(10)  # 10 Hz loop rate
    try:
        while rclpy.ok():
            node.get_logger().info("here")
            node.get_logger().info("Sending velocity command")
            node.send_velocity(1000.0, 1000.0, 1000.0)  # Move forward
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

