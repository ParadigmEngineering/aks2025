#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        
        # Publisher for velocity setpoints
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Timer to publish setpoints regularly
        self.timer = self.create_timer(0.1, self.publish_velocity_setpoint)  # 10 Hz
        
        # Service client to set flight mode
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.set_mode_ready = False
        self.timer_mode = self.create_timer(1.0, self.set_offboard_mode)  # Check/set mode every second
        
        # Initial velocity command
        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 1.0  # Move forward
        self.velocity_msg.angular.z = 0.0  # No rotation

    def publish_velocity_setpoint(self):
        self.get_logger().info('Publishing velocity setpoint...')
        self.velocity_pub.publish(self.velocity_msg)
    
    def set_offboard_mode(self):
        if not self.set_mode_ready and self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service is available.')
            self.set_mode_ready = True
        
        if self.set_mode_ready:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = 'OFFBOARD'
            
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.handle_mode_response)
    
    def handle_mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('OFFBOARD mode set successfully!')
            else:
                self.get_logger().error('Failed to set OFFBOARD mode.')
        except Exception as e:
            self.get_logger().error(f'SetMode call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
