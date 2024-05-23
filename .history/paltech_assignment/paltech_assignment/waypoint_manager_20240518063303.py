import rclpy
from rclpy.node import Node
from custom_interfaces.srv import GetWaypoints, SetWaypoints
from custom_interfaces.msg import Waypoint
from std_srvs.srv import Trigger
import json


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
        self.get_logger().info(f"Task 1: Loaded waypoints:  {self.waypoint_list_geo}")
        # COMPLETE YOUR CODE HERE
        response.success = True


            try:
        # Parse the request data
        geojson_file_path = request.data

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
            waypoint_msg.latitude = waypoint['geometry']['coordinates'][1]
            waypoint_msg.longitude = waypoint['geometry']['coordinates'][0]
            self.waypoint_list_geo.append(waypoint_msg)

        # If all the above steps are successful, set response.success to True
        response.success = True
        self.get_logger().info(f"Loaded waypoints:  {self.waypoint_list_geo}")

    except Exception as e:
        self.get_logger().error(f"Failed to load waypoints: {str(e)}")
        response.success = False

    return response

        return response

    def convert_waypoints_to_robot_frame(self):
        # COMPLETE YOUR CODE HERE
        self.get_logger().info(
            f" Task 2: Waypoints in robot frame:  {self.waypoint_list_robot_frame}"
        )

        pass

    def plot_waypoints(self):
        self.get_logger().info(
            f" Task 3: Plot and save a graph of loaded waypoints in robot coordinate frame (png)"
        )
        # COMPLETE YOUR CODE HERE
        pass

    def get_robot_waypoints_callback(self, request, response):
        response.waypoints = self.waypoint_list_geo

        return response

    def reset_waypoints_callback(self, request, response):
        response.waypoints = []

        return response

    def call_set_waypoints_geo(self, request):
        self.future = self.set_waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()

    set_waypoints_msg = SetWaypoints.Request()
    set_waypoints_msg.file_path = "/your_path/waypoints.geojson"
    response = waypoint_manager.call_set_waypoints_geo(set_waypoints_msg)
    if response.success == True:
        waypoint_manager.convert_waypoints_to_robot_frame()
        waypoint_manager.plot_waypoints()
    else:
        print("No waypoints loaded")

    rclpy.spin(waypoint_manager)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
