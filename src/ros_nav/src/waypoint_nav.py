#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import time
import os

class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        # Create a service client for pushing waypoints
        self.client = self.create_client(WaypointPush, '/mavros/mission/push')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create empty waypoints list and call function to fill it with waypoints
        self.waypoints = []
        self.create_waypoint_list()

        # Create a request with the waypoints
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = self.waypoints
        
        # self.temp_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        # self.temp_timer = self.create_timer(0.1, self.publish_velocity_setpoint)
        # self.temp_msg = Twist()

        # # Service client to set flight mode
        # self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        # self.set_mode_ready = False
        # self.timer_mode = self.create_timer(1.0, self.set_mode)

        # Start the mission
        #self.start_mission()

        self.get_logger().info("before service call")

        # Call the service to push waypoints
        response = self.client.call(request)

        self.get_logger().info("after service call")

        # if response.success:
        #     self.get_logger().info(f"Mission uploaded successfully with {len(request.waypoints)} waypoints.")
        # else:
        #     self.get_logger().error("Failed to upload mission.")
    
    def create_waypoint_list(self):
        package_share_path = get_package_share_directory('ros_nav')
        latitudes_path = package_share_path + "/data/latitudes.txt"
        longitudes_path = package_share_path + "/data/longitudes.txt"
        with open(latitudes_path) as latitudes, open(longitudes_path) as longitudes:
            for i, (latitude, longitude) in enumerate(zip(latitudes, longitudes)):
                latitude = float(latitude.strip())
                longitude = float(longitude.strip())
                self.waypoints.append(Waypoint(
                    frame=3,
                    command=16,
                    is_current=(i == 0),  # True for the first waypoint, False for others
                    autocontinue=True,
                    param1=0.0,
                    param2=0.0,
                    param3=0.0,
                    param4=0.0,
                    x_lat=latitude,
                    y_long=longitude,
                    z_alt=0.0
                ))

    def publish_velocity_setpoint(self):
        self.get_logger().info('Publishing velocity setpoint...')
        self.temp_pub.publish(self.temp_msg)

    def start_mission(self):
        client = self.create_client(SetMode, '/mavros/set_mode')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = SetMode.Request()
        request.base_mode = 1  # Change to match your vehicle's requirements
        request.custom_mode = "AUTO.MISSION"  # For PX4, or adjust as needed for ArduPilot

        response = client.call(request)
        if response.mode_sent:
            self.get_logger().info(f'Mission started: {response.mode_sent}')
            # Keep the timer active for a few more seconds
        else:
            self.get_logger().error("Failed to set mode")

    def set_mode(self):
        if not self.set_mode_ready and self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service is available.')
            self.set_mode_ready = True
        
        if self.set_mode_ready:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = 'AUTO.MISSION'
            
            future = self.set_mode_client.call_async(request)
            future.add_done_callback(self.handle_mode_response)

    def handle_mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Mode set successfully!')
            else:
                self.get_logger().error('Failed to set mode.')
        except Exception as e:
            self.get_logger().error(f'SetMode call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()