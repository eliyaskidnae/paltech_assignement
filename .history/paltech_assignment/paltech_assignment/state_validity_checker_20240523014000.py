
import numpy as np
import math

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""
    # Constructor
    def __init__(self,area_size ,  distance=0.2 ):
        # map: 2D array of integers which categorizes world occupancy
        self.obstacle_list = None 

        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # distance to be checked for collision
        self.distance = distance
        self.area_size = area_size  
        # world position of cell (0, 0) in self.map                      

    # Set occupancy map, its resolution and origin. 
    def set(self, data , area_size , tolorance = 0.01):
        '''Set the occupancy map, its resolution and origin.
        Args:
            data (numpy.ndarray): A 2D numpy array containing the obstacle list .
            tolorance (float): The distance to be checked for collision.
        '''
      
        self.obstacle_list = data
        self.tolorance = tolorance
        self.height = self.area_size
        self.width = self.area_size

    # computes distance between two positions 
    def get_distance( self , first_pose , second_pose):
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)  
    def is_valid(self, pose): 
        '''Returns True if the given position is valid and False otherwise.
        A position is valid if it is inside the map bounds and not in collision with any obstacle.
        
        Args:
            
            pose (tuple): A tuple containing the x and y coordinates of the position to be checked. 
            Returns:
            bool: True if the position is valid and False otherwise.
            '''
        if pose[0] < 0 or pose[1] < 0 or pose[0] > self.width or pose[1] > self.height:
            return False
        for obstacle in self.obstacle_list:
            if self.get_distance(pose, obstacle) < self.tolorance:
                return False


        return True
    def check_path_smooth(self, path):
        '''Returns True if the given path is valid and False otherwise.
        A path is valid if it is inside the map bounds and not in collision with any obstacle.
        
        Args:
            path (list): A list of tuples containing the x and y coordinates of the path to be checked. 
            Returns:
            bool: True if the path is valid and False otherwise.'''

        for pose in path:
            if not self.is_valid(pose):
                return False
        return True
    