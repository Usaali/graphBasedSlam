from world import world
from vehicle import vehicle
import time
from math import pi,sin,cos
plane = world(150,50)
veh = vehicle(plane,movementError= 1 , sensRange= 20)
veh.move(0,30)
time.sleep(2)
while(True):
    for i in range(10):
        step = pi/5 * i
        veh.move(60*(sin(step+pi/10)-sin(step)),60*(cos(step+pi/10)-cos(step)))
        time.sleep(0.2)
