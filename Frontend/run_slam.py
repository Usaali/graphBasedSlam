from vehicle import vehicle
from world import world
from slam import slam
from math import sin,cos,pi
import random
from pathlib import Path
#from backend.PoseGraph import PoseGraph
from numpy import arctan2
import os

def driveCircle(plane: world,veh: vehicle, s: slam, r: float, steps: int) -> None:
    """Lets the vehicle drive in a circle

    Args:
        plane (world): the world object to drive in
        veh (vehicle): the vehicle object
        s (slam): the slam object
        r (float): the radius of the circle
        steps (int): how many steps should be made to approximate the circle
    """
    for i in range(steps):
            step = pi/(steps/2) * i
            veh.move(r*(sin(step+pi/steps)-sin(step)),r*(cos(step+pi/steps)-cos(step)))
            print(s.slam_step())
            plane.plot(veh)

def driveRandom(plane: world,veh: vehicle, s: slam,steps: int, stepWidth: float):
    """Lets the Vehicle move in a random direction with a maximum angular change of 90°

    Args:
        plane (world): the world object to drive in
        veh (vehicle): the vehicle object
        s (slam): the slam object
        steps (int): how many steps should the vehicle make
        stepWidth (float): how big the steps are
    """
    rotation = 0
    for i in range(steps):
        rotation += random.random() * pi - pi/2
        while(not veh.move(stepWidth*sin(rotation),stepWidth*cos(rotation))):
            rotation += random.random() * 2 * pi - pi
        print(s.slam_step())
        plane.plot(veh)

def driveStraight(plane: world,veh: vehicle, s: slam, dx: int, dy: int):
    veh.move(dx,dy)
    print(s.slam_step())
    plane.plot(veh)

def backendData(slam):
    """Creates the output files for the Matlab Optimization
        NOTE: file_path should be the same directory as the main.m file of the Backend
    Args:
        s (slam): the slam object
    """
    file_path = "../Backend"
    if(not os.path.exists(file_path)):
        os.mkdir(file_path)
    truthFile = open(file_path+"/truth.txt","w")
    estFile = open(file_path+"/est.txt","w")
    landmarkFile = open(file_path+"/landmarks.txt","w")
    landmarkVectorFile = open(file_path+"/landmark_vec.txt","w")
    for i in range(len(slam.veh.true_path)):
        if(i == 0):
            truthFile.write(str(slam.veh.true_path[i][0])+" "+str(slam.veh.true_path[i][1])+" 0\n")
        else:
            truthFile.write(str(slam.veh.true_path[i][0])+" "+str(slam.veh.true_path[i][1])+" "+str(arctan2(slam.veh.true_path[i][1]-slam.veh.true_path[i-1][1],slam.veh.true_path[i][0]-slam.veh.true_path[i-1][0]))+"\n")
    for i in range(len(slam.veh.path)):
        if(i == 0):
            estFile.write(str(slam.veh.path[i][0])+" "+str(slam.veh.path[i][1])+" 0\n")
        else:
            estFile.write(str(slam.veh.path[i][0])+" "+str(slam.veh.path[i][1])+" "+str(arctan2(slam.veh.path[i][1]-slam.veh.path[i-1][1],slam.veh.path[i][0]-slam.veh.path[i-1][0]))+"\n")
    for i in range(slam.pos_marker+2,len(slam.z),2):
        landmarkFile.write(str(slam.z[i])+" "+str(slam.z[i+1])+" 0\n")

    for step in slam.landmark_array:
        for lm in step:
            landmarkVectorFile.write(str(lm[0])+" "+str(lm[1])+" "+str(lm[2])+" "+str(lm[3])+";")
        landmarkVectorFile.write("\n")

    truthFile.close()
    estFile.close()
    landmarkFile.close()
    landmarkVectorFile.close()
if __name__ == "__main__":
    plane = world(50,25)
    veh = vehicle(plane,movementError=2 , measuringError =0 , sensRange= 15)
    print(plane)
    print(veh)
    s = slam(veh)
    plane.plot(veh)

    while True:
        driveCircle(plane,veh,s,20,50)
        #driveStraight(plane,veh,s,1,0)
        #driveRandom(plane,veh,s,5,5)
        backendData(s)
        #s.plot_matrices()
        #s.get_error()
        input()
