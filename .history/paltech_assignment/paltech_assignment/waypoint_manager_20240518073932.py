import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetWaypoints, SetWaypoints
from custom_interfaces.msg import Waypoint
from std_srvs.srv import Trigger
import json
import math
import matplotlib.pyplot as plt
import numpy as np


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")
        self.get_waypoints_srv = self.create_service(
            GetWaypoints, "get_robot_waypoints", self.get_robot_waypoints_callback
        )
        self.set_waypoint_srv = self.create_service(
            SetWaypoints, "set_waypoints", self.set_waypoints_callback
        )
        self.reset_waypoint_srv = self.create_service(
            Trigger, "reset_waypoints", self.reset_waypoints_callback
        )
        self.robot_inital_geo = (47.740114, 10.322442)

        self.waypoint_list_geo = []  # needs to be an array of Waypoint() messages
        self.waypoint_list_robot_frame = []
        self.set_waypoint_client = self.create_client(SetWaypoints, "set_waypoints")

    def set_waypoints_callback(self, request, response):
        response.success = False
        # self.get_logger().info(f"Task 1: Loaded waypoints:  {self.waypoint_list_geo}")
        
        try:
                # Parse the request data
                geojson_file_path = request.file_path
                print(geojson_file_path)

                # Open and read the geojson file
                with open(geojson_file_path, 'r') as file:
                    geojson_data = json.load(file)

                # Parse the geojson file to extract the waypoints
                waypoints = geojson_data['features']


                # Clear the current waypoint list
                self.waypoint_list_geo.clear()

                # Convert the waypoints to the Waypoint message format
                for waypoint in waypoints:
                    waypoint_msg = Waypoint()
                    print(waypoint)
                    waypoint_msg.latitude = waypoint['geometry']['coordinates'][1]
                    waypoint_msg.longitude = waypoint['geometry']['coordinates'][0]
                    self.waypoint_list_geo.append(waypoint_msg)

                # If all the above steps are successful, set response.success to True
                # COMPLETE YOUR CODE HERE
                response.success = True
                self.get_logger().info(f"Loaded waypoints:  {self.waypoint_list_geo}")

        except Exception as e:
                self.get_logger().error(f"Failed to load waypoints: {str(e)}")
                response.success = False

           

        return response

    def convert_waypoints_to_robot_frame(self):
        # COMPLETE YOUR CODE HERE
          # Constants for conversion
        R = 6371e3  # Radius of the Earth in meters
        lat1_rad = math.radians(self.robot_inital_geo[0])  # Latitude of the robot's initial pose in radians
        lon1_rad = math.radians(self.robot_inital_geo[1])  # Longitude of the robot's initial pose in radians
        for waypoint in self.waypoint_list_geo:
            # Convert latitude and longitude to radians
            print(waypoint)
            lat2_rad = math.radians(waypoint.latitude)
            lon2_rad = math.radians(waypoint.longitude)
            
            # Calculate X and Y using the Equirectangular approximation
            X = R * (lon2_rad - lon1_rad) * math.cos(lat1_rad)
            Y = R * (lat2_rad - lat1_rad)

            # Append the transformed waypoint to the list
            self.waypoint_list_robot_frame.append((X, Y))

        self.get_logger().info(
            f" Task 2: Waypoints in robot frame:  {self.waypoint_list_robot_frame}"
        )
        

    def plot_waypoints(self):
        self.get_logger().info(
            f" Task 3: Plot and save a graph of loaded waypoints in robot coordinate frame (png)"
        )
        # COMPLETE YOUR CODE HERE

        # Extract X and Y coordinates from the waypoints
        X = [waypoint[0] for waypoint in self.waypoint_list_robot_frame]
        Y = [waypoint[1] for waypoint in self.waypoint_list_robot_frame]
        yaw = [0] * len(self.waypoint_list_robot_frame)

        # Plot the waypoints
        plt.figure()
        plt.quiver(X, Y, np.cos(yaw), np.sin(yaw))
        plt.scatter(X, Y)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Waypoints in Robot Frame")
        plt.grid()
        
        pass

    def get_robot_waypoints_callback(self, request, response):
        response.waypoints = self.waypoint_list_geo

        return response

    def reset_waypoints_callback(self, request, response):
        response.waypoints = []

        return response

    def call_set_waypoints_geo(self, request):
        print("Calling set_waypoints service")
        self.future = self.set_waypoint_client.call_async(request)
        print("Waiting for response" , self.future)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()

    set_waypoints_msg = SetWaypoints.Request()
    set_waypoints_msg.file_path = "/home/elias/ros2_foxy/src/paltech_assignment/paltech_assignment/waypoints/waypoints.geojson"
    response = waypoint_manager.call_set_waypoints_geo(set_waypoints_msg)
    if response.success == True:
        print("Waypoints loaded successfully")
        waypoint_manager.convert_waypoints_to_robot_frame()
        waypoint_manager.plot_waypoints()
    else:
        print("No waypoints loaded")

    rclpy.spin(waypoint_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
