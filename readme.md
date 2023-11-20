# Paltech Assigment

## Part 1: Basic waypoint manager in ROS2

### Task 1: Set waypoints from geojson file.
Complete the **set_waypoints_callback** function in **waypoint_manager.py** in order to load the waypoints to the node.
Complete the convert_waypoints_to_robot_frame function in waypoint_manager.py. This function should convert the list of waypoints from geographic coordinates to the robot's coordinate frame (X, Y), where:  
    X represents the distance in meters to the east of the robot's initial pose.
    Y represents the distance in meters to the north of the robot's initial pose.
### Task3: Loaded waypoints visualization 
Plot the waypoints (X,Y,yaw) in robot coordinate frame using matplotlib or another visualization tool. 

## Part 2: Path Planning 

This task consists in designing a path planning algorithm for the robot to efficiently follow waypoints: 

- Generate 500 random GPS points distributed in an area of 200 m². These points represent weed positions.
- Over the M points just created, add a cluster of approximately 2-5 plants around each point to simulate weed clusters. This can be achieved using a normal distribution with a radius of 5 meters.
- Starting from an initial robot position, sort the generated points in a way that maximizes the number of plants processed in the shortest possible time.
- Calculate the arrival angle at each waypoint to avoid excessively sharp curves during waypoint following.
- Plot the calculated path, connecting the waypoints with lines to represent the travel route.


Comments:

- The robot is non-holonomic and follows a Dubins motion model with a minimum turning radius of 2 meters.
- It is not necessary for the robot to reach every generated point. Points that are unreachable due to the robot's turning radius constraints can be omitted from the plan.

BONUS:

- Integrate your path planning code into a ROS2 node that calls the service to retrieve waypoints from the waypoint manager and publishes the calculated path.
- How does the path planning change if we allow the robot to move in reverse? (REED_SHEPP motion model instead of DUBINS)



To submit your completed assignment, **please fork this repository**, complete the tasks, and then push your changes to your fork. Once done, send us the link to your forked repository at felix.shiegg@paltech.eu with the subject line "Assignment Submission - [Your Name]". Ensure your repository is public so we can review your work.










