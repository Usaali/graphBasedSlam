import random as rand
from vehicle import vehicle
import seaborn
import matplotlib.pyplot as plt

class world:
    def __init__(self,size: int, num_landmarks: int):
        self.size = size
        self.num_landmarks = num_landmarks
        self.landmarks = []
        self.createLandmarks()
        plt.ion()
        seaborn.set_style("whitegrid")
        plt.switch_backend("TKAgg")
        plt.figure()
    
    def getLandmarks(self):
        return self.landmarks

    def createLandmarks(self):
        for i in range(self.num_landmarks):
            self.landmarks.append([round(rand.random()*self.size),round(rand.random()*self.size)])

    def plot(self, v: vehicle):
        plt.clf()
        axes = plt.gca()
        #axes.text(v.pos[0],v.pos[1], 'x', ha='center', va='center', color='r', fontsize=15)
        for l in self.landmarks:
            plt.plot(l[0],l[1],marker="D",color = "b")
        for l in v.get_detected():
            plt.plot(l[0],l[1],marker="D",color = "y")
        plt.plot(*zip(*v.path),color = "red")
        plt.plot(*zip(*v.true_path),color = "black")
        plt.plot(v.pos[0],v.pos[1],marker="^",color = 'r')
        plt.plot(v.true_pos[0],v.true_pos[1],marker="^",color = 'black')
        axes.set_xticks([x for x in range(self.size+1)],minor = True)
        axes.set_yticks([y for y in range(self.size+1)],minor = True)
        plt.draw()
        plt.pause(0.001)