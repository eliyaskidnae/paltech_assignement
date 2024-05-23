
import numpy as np
import math

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.2 ):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # distance to be checked for collision
        self.distance = distance
        # world position of cell (0, 0) in self.map                      
      
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        global local_map, local_origin, local_resolution
        self.obstacle_list = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]

        

    # computes distance between two positions 
    def get_distance( self , first_pose , second_pose):
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)
    
    # Given a pose, returs true if the pose is not in collision and false othewise.

    def is_valid(self, pose): 

        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as invalid.
       
        # check if point is in obstacle list 
        if pose[0] < 0 or pose[1] < 0 or pose[0] > self.width or pose[1] > self.height:
            return False
        not_valid = 


        return True