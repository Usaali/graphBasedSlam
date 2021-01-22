from vehicle import vehicle
from world import world
from slam import slam
from math import sin,cos,pi
import random

def driveCircle(plane: world,veh: vehicle, s: slam, r: float, steps: int) -> None:
    for i in range(steps):
            step = pi/(steps/2) * i
            veh.move(r*(sin(step+pi/steps)-sin(step)),r*(cos(step+pi/steps)-cos(step)))
            s.slam_step()
            print(s.get_estimate())
            plane.plot(veh)

def driveRandom(plane: world,veh: vehicle, s: slam,steps: int, stepWidth: float):
    rotation = 0
    for i in range(steps):
        rotation += random.random() * 2 * pi - pi
        veh.move(stepWidth*sin(rotation),stepWidth*cos(rotation))
        s.slam_step()
        print(s.get_estimate())
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
        input()
