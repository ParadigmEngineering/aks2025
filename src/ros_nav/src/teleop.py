#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode
from std_srvs.srv import Trigger
import threading
import subprocess


class TeleoperationNode(Node):
    def __init__(self):
        super().__init__('teleoperation_node')
        
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
        self.velocity_msg.linear.y = 0.0  # No forward movement
        self.velocity_msg.angular.z = 0.0  # No rotation

        # Start a thread to read user input for teleoperation
        self.input_thread = threading.Thread(target=self.run)
        self.input_thread.start()

        # Variable to assign autonomous node to and flag for what mode we are in
        self.autonomous_node_process = None
        self.autonomous = False

        # Create client for waypoint shutdown service
        self.client = self.create_client(Trigger, 'shutdown')

    def publish_velocity_setpoint(self):
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
        except Exception as e:
            self.get_logger().error(f'SetMode call failed: {e}')

    def run(self):
        try:
            while rclpy.ok():
                key = input("Enter command (w/a/s/d/x): ").strip()
                if key == 'w' and not self.autonomous:  # Move forward
                    self.velocity_msg.linear.y = 1.0
                    self.velocity_msg.linear.x = 0.0
                elif key == 's' and not self.autonomous:  # Move backward doesn't work though
                    self.velocity_msg.linear.y = -1.0
                    self.velocity_msg.linear.x = 0.0
                elif key == 'a' and not self.autonomous:  # Turn left
                    self.velocity_msg.linear.y = 0.0
                    self.velocity_msg.linear.x = -1.0
                elif key == 'd' and not self.autonomous:  # Turn right
                    self.velocity_msg.linear.y = 0.0
                    self.velocity_msg.linear.x = 1.0
                elif key == 'x' and not self.autonomous:  # Stop
                    self.velocity_msg.linear.y = 0.0
                    self.velocity_msg.linear.x = 0.0
                elif key == 'z': # Toggle autonomous
                    if not self.autonomous:
                        self.start_autonomous()
                    else:
                        self.end_autonomous()
                else:
                    self.get_logger().warn(f"Unrecognized key input: {key}")
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down teleoperation node.")

    def start_autonomous(self):
        # Kill timer that sends velocity setpoints
        self.destroy_timer(self.timer)
        self.timer = None

        # Start waypoint mission
        if self.autonomous_node_process is None:
            self.autonomous_node_process = subprocess.Popen(
                ['ros2', 'run', 'ros_nav', 'waypoint']
            )
            self.autonomous = True

    def end_autonomous(self, kill=False):
        # Start velocity timer again
        if not kill:
            self.timer = self.create_timer(0.1, self.publish_velocity_setpoint)

        # End waypoint mission
        if self.autonomous_node_process is not None:
            # Send service call to kill node
            if not self.client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('Shutdown service not available.')
                return
            
            request = Trigger.Request()
            future = self.client.call_async(request)

            # Add a callback to handle the service response asynchronously
            future.add_done_callback(self.shutdown_callback)

            # Instead of spinning, we allow other callbacks to run
            self.get_logger().info("Waiting for shutdown service response...")

    def shutdown_callback(self, future):
        # Call shutdown service
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f'Shutdown service response: {result.message}')
            else:
                self.get_logger().error('Shutdown service call failed.')
        except Exception as e:
            self.get_logger().error(f'Failed to call shutdown service: {e}')

        # Kill subprocess for waypoint mission after shutdown
        if self.autonomous_node_process is not None:
            self.autonomous_node_process.terminate()
            self.autonomous_node_process.wait()
            self.autonomous_node_process = None
            self.autonomous = False
        
        self.velocity_msg.linear.y = 0.0
        self.velocity_msg.linear.x = 0.0

    def get_autonomous(self):
        return self.autonomous


 
def main(args=None):
    rclpy.init(args=args)
    node = TeleoperationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user. Shutting down.")
    finally:
        # Kill autonomous mode if active
        if node.get_autonomous():
            node.end_autonomous(kill=True)
        # then shutdown
        rclpy.shutdown()
        node.input_thread.join()  # Ensure the input thread joins when the node is shut down

if __name__ == '__main__':
    main()
