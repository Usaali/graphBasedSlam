import random as rand
import numpy as np
from math import pow,sqrt
class vehicle:

    def __init__(self, plane,  movementError: float = 1, measuringError: float = 1, sensRange: float = 10):
        """A vehicle object, that can move and sense in a world

        Args:
            plane (world.world): The world to spawn the vehicle in
            movementError (float, optional): The error that is added on any movement. Defaults to 1.
            measuringError (float, optional): The error that is added on any measurement. Defaults to 1.
            sensRange (float, optional): The range of the vehicle sensors. Defaults to 50.
        """
        self.world = plane
        self.plane_size = plane.size
        self.true_pos = np.array([self.plane_size/2,self.plane_size/2])
        self.pos = np.array([self.plane_size/2,self.plane_size/2])
        self.sense_range = sensRange
        self.movement_error = movementError
        self.measuring_error = measuringError
        self.detected_landmarks = None
        self.path = np.array([[self.plane_size/2,self.plane_size/2]]) #path with first element absolute and all others as relative poses
        self.true_path = np.array([[self.plane_size/2,self.plane_size/2]])
        self.err_path = np.array([[self.plane_size/2,self.plane_size/2]])

    
    def rand(self, mean: float, std: float) -> float:
        """Creates a gaussian random value

        Args:
            mean (float): The mean of the gaussian curve
            std (float): The standard derivation of the gaussian curve

        Returns:
            float: The random value
        """
        return rand.gauss(mean,std)

    def get_detected(self):
        """returns a list of detected landmarks

        Returns:
            [[float, float]]: the currently detected landmarks 
        """
        return self.detected_landmarks

    def move(self,dx: float,dy: float) -> bool:
        """Moves the vehicle

        Args:
            dx (float): The delta x value to move
            dy (float): The delta y value to move

        Returns:
            (bool): False if movement would move the vehicle out of the world
        """
        self.true_pos[0] += dx
        self.true_pos[1] += dy
        dx = self.rand(dx, self.movement_error)
        dy = self.rand(dy, self.movement_error)
        self.pos[0] = dx
        self.pos[1] = dy
        self.err_path = np.append(self.err_path, [[self.err_path[-1][0]+dx,self.err_path[-1][1]+dy]],axis = 0)
        self.true_path = np.append(self.true_path,[[*self.true_pos]],axis=0)
        self.sense()
        #self.world.plot(self)
        return True
    
    def get_manhatten(self, pos) -> float:
        """Returns the manhatten distance to a point

        Args:
            pos (List[float]): The point to measure the distance to

        Returns:
            (float): The manhatten distance
        """
        return abs(self.true_pos[0]-pos[0]) + abs(self.true_pos[1] - pos[1])

    def get_euklidean(self,pos) -> float:
        """Returns the euklidean distance to a point

        Args:
            pos (List[float]): The point to measure the distance to

        Returns:
            (float): The euklidean distance
        """
        return sqrt(pow((self.true_pos[0]-pos[0]),2)+pow((self.true_pos[1]-pos[1]),2))

    def sense(self):
        """Detects if any Landmarks are in reach
        """
        temp = []
        for l in self.world.getLandmarks():
            if self.get_euklidean(l) <= self.sense_range :
                temp.append([self.rand(l[0]-self.true_pos[0],self.measuring_error),self.rand(l[1]-self.true_pos[1],self.measuring_error),l[2]])
        self.detected_landmarks = np.array(temp)