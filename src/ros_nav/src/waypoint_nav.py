#!/usr/bin/env python3

import rclpy
import mavros
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from mavros_msgs.srv import WaypointPush, WaypointClear, SetMode
from mavros_msgs.msg import Waypoint, WaypointList
from ament_index_python.packages import get_package_share_directory
import os

class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        # client_cb_group = ReentrantCallbackGroup()
        # timer_cb_group = client_cb_group

        # Create a service client for pushing waypoints
        # self.client = self.create_client(WaypointPush, '/mavros/mission/push', callback_group=client_cb_group)
        # self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

        client = mavros.Client()
        self.client = self.create_client(WaypointPush, '/mavros/mission/push')

        #Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create empty waypoints list and call function to fill it with waypoints
        self.waypoints_list = []
        #self.waypoints_list = WaypointList()
        self.create_waypoint_list()


        # Create a request with the waypoints
        request = WaypointPush.Request()
        request.start_index = 0
        #request.waypoints = self.waypoints
        #request.waypoints = self.waypoints_list.waypoints
        for waypoint in self.waypoints_list:
            request.waypoints.append(waypoint)

        
        self.get_logger().info("before call")

        # Call the service to push waypoints
        response = self.client.call(request)

        self.get_logger().info("aftercall")

        if response.success:
            self.get_logger().info(f"Mission uploaded successfully with {len(request.waypoints)} waypoints.")
        else:
            self.get_logger().error("Failed to upload mission.")

        # Start the mission
        self.start_mission()

    
    def create_waypoint_list(self):
        package_share_path = get_package_share_directory('ros_nav')
        latitudes_path = package_share_path + "/data/latitudes.txt"
        longitudes_path = package_share_path + "/data/longitudes.txt"
        with open(latitudes_path) as latitudes, open(longitudes_path) as longitudes:
            for latitude, longitude in zip(latitudes, longitudes):
                latitude = float(latitude.strip())
                longitude = float(longitude.strip())
                waypoint = Waypoint()
                waypoint.is_current = False
                if len(self.waypoints_list) == 0:
                    waypoint.is_current = True
                waypoint.frame=6
                waypoint.command=16
                waypoint.autocontinue = True
                waypoint.param1 = float(0)
                waypoint.param2 = float(1)
                waypoint.param3 = float("nan")
                waypoint.param4 = float("nan")
                waypoint.x_lat = latitude
                waypoint.y_long = longitude
                waypoint.z_alt = float(0)
                self.waypoints_list.append(waypoint)


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
        else:
            self.get_logger().error("Failed to set mode")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


            