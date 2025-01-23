#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode


class SetModeClient(Node):
    def __init__(self):
        super().__init__('set_mode_client')
        self.client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for SetMode service...')
        self.get_logger().info('SetMode service is available.')

    def change_mode(self, mode):
        request = SetMode.Request()
        request.base_mode = 0  # Not used when setting a custom mode
        request.custom_mode = mode

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Successfully changed mode to {mode}')
            else:
                self.get_logger().error(f'Failed to change mode to {mode}')
        else:
            self.get_logger().error('Service call failed!')


def main(args=None):
    rclpy.init(args=args)
    set_mode_client = SetModeClient()
    try:
        set_mode_client.change_mode('OFFBOARD')
    except KeyboardInterrupt:
        pass
    finally:
        set_mode_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
