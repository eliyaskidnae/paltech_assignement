# complete your code here
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import math

class PathPlanner:
    def __init__(self, area_size=1000, num_points=500, cluster_percentage=0.5, cluster_std=3, min_turning_radius=2):
        self.area_size = area_size
        self.num_points = num_points
        self.cluster_percentage = cluster_percentage
        self.cluster_std = cluster_std
        self.min_turning_radius = min_turning_radius
        self.robot_start_pos = None
        self.waypoints = None
        self.path_length = None
        self.path_time = None

    def generate_random_points(self):
        self.waypoints = np.random.uniform(0, self.area_size, (self.num_points, 2))

    def add_clusters(self):
        num_clusters = int(self.num_points * self.cluster_percentage)
        cluster_indices = np.random.choice(range(self.num_points), num_clusters, replace=False)

        for idx in cluster_indices:
            num_plants = np.random.randint(2, 11)
            cluster_center = self.waypoints[idx]
            plants_x = np.random.normal(cluster_center[0], self.cluster_std, num_plants)
            plants_y = np.random.normal(cluster_center[1], self.cluster_std, num_plants)
            cluster_points = np.column_stack((plants_x, plants_y))
            self.waypoints = np.concatenate((self.waypoints, cluster_points))

    def assign_start_position(self):
        self.robot_start_pos = np.random.uniform(0, self.area_size, 2)

    def dubins_path_planning(self):
        reachable_points = []
        for point in self.waypoints:
            distance = math.sqrt((point[0] - self.robot_start_pos[0])**2 + (point[1] - self.robot_start_pos[1])**2)
            if distance >= 2 * self.min_turning_radius:
                reachable_points.append(point)

        self.waypoints = np.array(reachable_points)

    def plot_path(self):
        sorted_indices = np.argsort(np.linalg.norm(self.waypoints - self.robot_start_pos, axis=1))
        sorted_waypoints = self.waypoints[sorted_indices]

        plt.figure(figsize=(10, 10))
        plt.scatter(self.waypoints[:, 0], self.waypoints[:, 1], color='blue', label='Waypoints')
        plt.scatter(self.robot_start_pos[0], self.robot_start_pos[1], color='red', label='Start Position')
        plt.plot(sorted_waypoints[:, 0], sorted_waypoints[:, 1], color='green', label='Path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Path Planning')
        plt.legend()
        plt.grid(True)
        plt.show()

    def calculate_path_length(self):
        self.path_length = 0
        if len(self.waypoints) > 1:
            # Sort the waypoints by their distance from the start position to create a simple path
            sorted_indices = np.argsort(np.linalg.norm(self.waypoints - self.robot_start_pos, axis=1))
            sorted_waypoints = self.waypoints[sorted_indices]

            # Calculate the path length
            for i in range(len(sorted_waypoints) - 1):
                distance = np.linalg.norm(sorted_waypoints[i + 1] - sorted_waypoints[i])
                self.path_length += distance

        self.path_time = self.path_length / 1  # Assuming constant velocity of 1 m/s

    def execute(self):
        self.generate_random_points()
        # self.add_clusters()
        self.assign_start_position()
        self.dubins_path_planning()
        self.plot_path()
        self.calculate_path_length()
        print(f"Path Length: {self.path_length:.2f} meters")
        print(f"Path Time: {self.path_time:.2f} seconds")

if __name__ == "__main__":
    path_planner = PathPlanner()
    path_planner.execute()
