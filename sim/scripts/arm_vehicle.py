#!/usr/bin/env python3

from mavros_msgs.srv import SetMode, CommandBool
import rclpy
from rclpy.node import Node

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')

    def set_offboard_mode(self):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service mavros/set_mode...')
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req)
        self.get_logger().info('Offboard mode set.')

    def arm_vehicle(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service mavros/cmd/arming...')
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.get_logger().info('Arming the vehicle.')

def main():
    rclpy.init()
    node = OffboardControl()
    node.arm_vehicle()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
