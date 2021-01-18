from world import world
from vehicle import vehicle
import numpy as np
import time
from math import pi,sin,cos,floor
import seaborn
import matplotlib.pyplot as plt
from pandas import DataFrame

class slam:
    def __init__(self, veh: vehicle, start = None) -> None:
        self.veh = veh
        self.num_landmarks = self.veh.world.num_landmarks
        self.world_size = self.veh.world.size
        
        #initialize constraints for starting point
        self.omega = np.zeros((2+2*self.num_landmarks,2+2*self.num_landmarks))
        self.eta = np.zeros((2+2*self.num_landmarks))

        self.omega[0][0] = 1
        self.omega[1][1] = 1


        if(start is None):
            self.eta[0] = self.world_size/2
            self.eta[1] = self.world_size/2
        else:
            self.eta[0] = start[0]/2
            self.eta[1] = start[1]/2
        
        self.veh.sense()
        measurement_factor = 1
        movement_factor = 1

        for landmark in self.veh.get_detected():
            l_num = landmark[2] #number of landmark
            l_meas_x = landmark[0] #measured x distance
            l_meas_y = landmark[1]

            #positions in the matrix
            l_x = int(2 + l_num*2)
            l_y = l_x + 1

            self.omega[0][0] += measurement_factor
            self.omega[l_x][l_x] += measurement_factor
            self.omega[0][l_x] -= measurement_factor
            self.omega[l_x][0] -= measurement_factor
            
            self.omega[1][1] += measurement_factor
            self.omega[l_y][l_y] += measurement_factor
            self.omega[1][l_y] -= measurement_factor
            self.omega[l_y][1] -= measurement_factor
            
            self.eta[0] -= l_meas_x/movement_factor
            self.eta[1] -= l_meas_y/movement_factor
            self.eta[l_x] += l_meas_x/movement_factor
            self.eta[l_y] += l_meas_y/movement_factor
        

    def plot_matrices(self):
        labels = []
        landmark_zero = len(self.omega[0])-2*self.num_landmarks #position of the first landmark entry
        for i in range(len(self.omega)):
            if(i<(len(self.omega)- 2 * self.num_landmarks)):
                if(i%2 == 0):
                    labels.append("X"+str(floor(i/2)))
                else:
                    labels.append("Y"+str(floor(i/2)))
            else:
                if(i%2 == 0):
                    labels.append("L"+str(floor((i-landmark_zero)/2))+"_x")
                else:
                    labels.append("L"+str(floor((i-landmark_zero)/2))+"_y")

        fig,(ax1, ax2) = plt.subplots(ncols=2)
        
        seaborn.heatmap(DataFrame(self.omega), cmap='Blues', annot=True, linewidths=.5, ax= ax1, xticklabels=labels, yticklabels=labels)
        seaborn.heatmap(DataFrame(self.eta), cmap='Blues', annot=True, linewidths=.5, ax = ax2, yticklabels=labels)
        plt.pause(0.01)
    
    def extend_matrices(self):
        """[summary]

        Args:
            omega (np.array): [description]
            eta (np.array): [description]
        """
        self.omega = np.insert(self.omega, len(self.omega)-2*self.num_landmarks, 0, axis = 0)
        self.omega = np.insert(self.omega, len(self.omega)-2*self.num_landmarks, 0, axis = 0)
        self.omega = np.insert(self.omega, len(self.omega[0])-2*self.num_landmarks, 0, axis = 1)
        self.omega = np.insert(self.omega, len(self.omega[0])-2*self.num_landmarks, 0, axis = 1)

        self.eta = np.insert(self.eta, len(self.eta)-2*self.num_landmarks, 0, axis = 0)
        self.eta = np.insert(self.eta, len(self.eta)-2*self.num_landmarks, 0, axis = 0)

    def slam_step(self):

        #a step from X_i to X_j
        #indices                                                                                                                                                for the matrix
        x_i = len(self.omega[0]) - 2*self.num_landmarks - 2
        y_i = x_i + 1
        self.extend_matrices()
        x_j = y_i + 1
        y_j = x_j + 1

        #movement in this step
        dx = self.veh.pos[0]
        dy = self.veh.pos[1]

        if self.veh.measuring_error == 0:
            measurement_factor = 1
        else:
            measurement_factor = 1.0/self.veh.measuring_error
        if self.veh.movement_error == 0:
            movement_factor = 1
        else:
            movement_factor = 1.0/self.veh.movement_error

        movement_factor = 1
        measurement_factor = 1

        #Measurements

        self.omega[x_i][x_i] += movement_factor
        self.omega[x_j][x_j] += movement_factor
        self.omega[x_i][x_j] -= movement_factor
        self.omega[x_j][x_i] -= movement_factor
        
        self.omega[y_i][y_i] += movement_factor
        self.omega[y_j][y_j] += movement_factor
        self.omega[y_i][y_j] -= movement_factor
        self.omega[y_j][y_i] -= movement_factor

        self.eta[x_i] -= dx/movement_factor
        self.eta[y_i] -= dy/movement_factor
        self.eta[x_j] += dx/movement_factor
        self.eta[y_j] += dy/movement_factor
        
        landmark_zero = len(self.omega[0])-2*self.num_landmarks #position of the first landmark entry
        for landmark in self.veh.get_detected():
            l_num = landmark[2] #number of landmark
            #print("Landmark number: "+str(l_num)+"\n")
            l_meas_x = landmark[0] #measured x distance
            l_meas_y = landmark[1]

            #positions in the matrix
            l_x = int(landmark_zero + l_num*2)
            l_y = l_x + 1

            self.omega[x_j][x_j] += measurement_factor
            self.omega[l_x][l_x] += measurement_factor
            self.omega[x_j][l_x] -= measurement_factor
            self.omega[l_x][x_j] -= measurement_factor
            
            self.omega[y_j][y_j] += measurement_factor
            self.omega[l_y][l_y] += measurement_factor
            self.omega[y_j][l_y] -= measurement_factor
            self.omega[l_y][y_j] -= measurement_factor
            
            self.eta[x_j] -= l_meas_x/movement_factor
            self.eta[y_j] -= l_meas_y/movement_factor
            self.eta[l_x] += l_meas_x/movement_factor
            self.eta[l_y] += l_meas_y/movement_factor

        return self.get_estimate()

    def get_estimate(self):
        #the matrix might have landmarks in it that never got detected, making it singular
        remove_count = 0
        om = np.copy(self.omega)
        et = np.copy(self.eta)
        undetected = []
        for i in range(len(om)):
            if(om[i][i] == 0):
                undetected.append(i)
        for i in reversed(undetected):
            om = np.delete(om,i,0)
            om = np.delete(om,i,1)
            et = np.delete(et,i,0)
            remove_count += 1
        print(om)
        z = np.matmul(np.linalg.inv(om), et)
        pos = len(om) - 2*(self.num_landmarks-remove_count) - 2
        est_pos = [z[pos],z[pos+1]]
        self.veh.path =  np.append(self.veh.path,[[*est_pos]],axis=0)
        return [*est_pos]
        #plt.figure()
        #seaborn.heatmap(DataFrame(z), cmap='Reds', annot=True, linewidths=.5)
        #plt.pause(0.01)        

if __name__ == "__main__":
    plane = world(50,20)
    veh = vehicle(plane,movementError= 2, measuringError = 1 , sensRange= 20)
    #time.sleep(2)
    s = slam(veh)
    while True:
        #veh.move(0,5)
        #print(s.slam_step())
        #plane.plot(veh)
        #s.plot_matrices()
        for i in range(10):
            step = pi/5 * i
            veh.move(20*(sin(step+pi/10)-sin(step)),20*(cos(step+pi/10)-cos(step)))
            s.slam_step()
            print(s.get_estimate())
            time.sleep(0.5)
            plane.plot(veh)
        #s.plot_matrices()
        #print(s.omega)
        print(s.eta)
        x = 0
        y = 0
        input()
    #s.get_estimate(1)
    input()
    veh.move(20,0)
    s.slam_step()
    s.plot_matrices()
    #s.get_estimate(1)
    input()
    exit()
    while(True):
        for i in range(10):
            step = pi/5 * i
            veh.move(60*(sin(step+pi/10)-sin(step)),60*(cos(step+pi/10)-cos(step)))
            time.sleep(0.2)
