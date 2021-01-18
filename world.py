import random as rand
from vehicle import vehicle
import seaborn
import matplotlib.pyplot as plt
import numpy as np

class world:
    def __init__(self,size: int, num_landmarks: int):
        """A square world object that can inhabit vehicles

        Args:
            size (int): The world size
            num_landmarks (int): The amount of landmarks in the world
        """
        self.size = size
        self.num_landmarks = num_landmarks
        self.landmarks = None
        self.createLandmarks()
        plt.ion()
        seaborn.set_style("whitegrid")
        plt.switch_backend("Qt5Agg")
        self.fig = plt.figure()
    
    def getLandmarks(self):
        """Returns all landmarks

        Returns:
            List[List[float]]: a list of landmarks
        """
        return self.landmarks

    def createLandmarks(self):
        """populates the world with randomly placed landmarks
        """
        temp = []
        for i in range(self.num_landmarks):
            temp.append([round(rand.random()*self.size),round(rand.random()*self.size),i])
        self.landmarks = np.array(temp)

    def plot(self, v: vehicle):
        """Displays/Updates a plot with all landmarks and the robot path 

        Args:
            v (vehicle): The vehicle to include in the plot
        """
        self.fig.clf()
        axes = plt.gca()
        #axes.text(v.pos[0],v.pos[1], 'x', ha='center', va='center', color='r', fontsize=15)
        for l in self.landmarks:
            plt.plot(l[0],l[1],marker="D",color = "b")
        for l in v.get_detected():
            plt.plot(v.path[-1][0]+l[0],v.path[-1][1]+l[1],marker="D",color = "y")
        plt.plot(*zip(*v.true_path),color = "black")
        plt.plot(*zip(*v.err_path),color = "yellow")
        plt.plot(*zip(*v.path),color = "red",)

        plt.plot(v.path[-1][0],v.path[-1][1],marker="^",color = 'r')
        plt.plot(v.true_pos[0],v.true_pos[1],marker="^",color = 'black')
        circ = plt.Circle((v.true_pos[0],v.true_pos[1]), v.sense_range, color='blue' , fill=False)
        axes.add_patch(circ)
        axes.set_xticks([x for x in range(self.size+1)],minor = True)
        axes.set_yticks([y for y in range(self.size+1)],minor = True)
        plt.draw()
        plt.pause(0.001)