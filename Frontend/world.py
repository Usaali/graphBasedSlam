import random as rand
from vehicle import vehicle
import seaborn
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MultipleLocator
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
        self.createLandmarks()
        plt.ion()
        seaborn.set_style("whitegrid")
        seaborn.set_context("paper")
        #plt.switch_backend("Qt5Agg")
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
        for l in self.landmarks:
            lm, = plt.plot(l[0],l[1],marker="D",color = "b")
        for l in v.get_detected():
            dlm, = plt.plot(v.path[-1][0]+l[0],v.path[-1][1]+l[1],marker="D",color = "y")
        if(len(self.landmarks) > 0):
            lm.set_label("landmarks")
        if(len(v.get_detected()) > 0):
            dlm.set_label("detected landmarks")
        plt.plot(*zip(*v.true_path),color = "black", label = "groundtruth")
        plt.plot(*zip(*v.err_path),color = "green", label = "raw trajectory with noise")
        plt.plot(*zip(*v.path),color = "red", label = "calculated trajectory")

        plt.plot(v.path[-1][0],v.path[-1][1],marker="^",color = 'r', label = "estimated positon")
        plt.plot(v.true_pos[0],v.true_pos[1],marker="^",color = 'black', label="actual position")
        circ = plt.Circle((v.true_pos[0],v.true_pos[1]), v.sense_range, color='blue' , fill=False, label = "sensing range")
        axes.add_patch(circ)

        major_ticks = np.arange(0, self.size+1, 10)
        minor_ticks = np.arange(0, self.size+1, 5)

        axes.set_xticks(major_ticks)
        axes.set_xticks(minor_ticks, minor=True)
        axes.set_yticks(major_ticks)
        axes.set_yticks(minor_ticks, minor=True)
        axes.grid(which='minor', alpha=0.5)
        axes.grid(which='major', alpha=1)
        plt.draw()
        plt.tight_layout()
        axes.legend(loc="upper right")
        plt.pause(0.001)

    def __repr__(self):
        string = "A world with size "+str(self.size)+" containg "+str(self.num_landmarks)+" landmarks\n"
        for i in self.landmarks:
            string += "Landmark number "+str(int(i[2]))+" at "+str(i[0])+", "+str(i[1])+"\n"
        return string
