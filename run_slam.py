from vehicle import vehicle
from world import world
from slam import slam
from math import sin,cos,pi
import random

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

if __name__ == "__main__":
    plane = world(100,100)
    veh = vehicle(plane,movementError= 1, measuringError = 2 , sensRange= 20)
    print(plane)
    print(veh)
    s = slam(veh)
    while True:
        #driveCircle(plane,veh,s,20,100)
        driveRandom(plane,veh,s,10,10)
        s.get_error()
        input()
