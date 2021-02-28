from world import world
from vehicle import vehicle

import numpy as np
from numpy import arctan2
import time
import os
from math import pi,sin,cos,floor,sqrt,pow
import seaborn
import matplotlib.pyplot as plt
from pandas import DataFrame

class slam:
    def __init__(self, veh: vehicle, start = None) -> None:
        self.veh = veh
        self.num_landmarks = self.veh.world.num_landmarks
        self.world_size = self.veh.world.size
        
        self.pos_marker = 0

        #initialize constraints for starting point
        self.omega = np.zeros((2+2*self.num_landmarks,2+2*self.num_landmarks))
        self.eta = np.zeros((2+2*self.num_landmarks))

        ## added by Bian
        self.landmark_array = []
        self.z = np.array([])  # x1, y1, x2, y2,...lx1, ly1, .....
        ###

        self.omega[0][0] = 1
        self.omega[1][1] = 1

        #initialize the starting vector
        if(start is None):
            self.eta[0] = self.world_size/2
            self.eta[1] = self.world_size/2
        else:
            self.eta[0] = start[0]/2
            self.eta[1] = start[1]/2
        
        self.veh.sense()
        measurement_factor = 1
        movement_factor = 1
        tempStateVec = []
        # update the matrices with the data of the sensor
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

            l_dist = sqrt(pow(l_meas_x,2)+pow(l_meas_y,2))
            phi = self.veh.pi2pi(arctan2(l_meas_y,l_meas_x))
            tempStateVec.append([l_dist, phi-self.veh.phi, phi, l_num])
        self.landmark_array.append(tempStateVec)


    def plot_matrices(self):
        """Plots the matrix omega and the vector as a heatmap for easier visualisation
        """
        labels = []
        landmark_zero = len(self.omega[0])-2*self.num_landmarks #position of the first landmark entry
        for i in range(len(self.omega)):
            if(i<(len(self.omega)- 2 * self.num_landmarks)):
                if(i%2 == 0):
                    labels.append("T"+str(floor(i/2))+"_x")
                else:
                    labels.append("T"+str(floor(i/2))+"_y")
            else:
                if(i%2 == 0):
                    labels.append("L"+str(floor((i-landmark_zero)/2))+"_x")
                else:
                    labels.append("L"+str(floor((i-landmark_zero)/2))+"_y")

        fig,(ax1, ax2) = plt.subplots(ncols=2,gridspec_kw={'width_ratios': [3, 1]})
        
        seaborn.heatmap(DataFrame(self.omega), cmap='Blues', annot=True, linewidths=.5, ax= ax1, xticklabels=labels, yticklabels=labels)
        seaborn.heatmap(DataFrame(self.eta), cmap='Blues', annot=True, linewidths=.5, ax = ax2, xticklabels=[""] ,yticklabels=labels)
        ax1.set_title("Omega")
        ax2.set_title("Eta")
        plt.tight_layout()
        plt.pause(0.01)
    

    def extend_matrices(self):
        """This is a helper function to increase the dimension of the arrays
        """
        self.omega = np.insert(self.omega, len(self.omega)-2*self.num_landmarks, 0, axis = 0)
        self.omega = np.insert(self.omega, len(self.omega)-2*self.num_landmarks, 0, axis = 0)
        self.omega = np.insert(self.omega, len(self.omega[0])-2*self.num_landmarks, 0, axis = 1)
        self.omega = np.insert(self.omega, len(self.omega[0])-2*self.num_landmarks, 0, axis = 1)

        self.eta = np.insert(self.eta, len(self.eta)-2*self.num_landmarks, 0, axis = 0)
        self.eta = np.insert(self.eta, len(self.eta)-2*self.num_landmarks, 0, axis = 0)

    def slam_step(self):
        """This describes a step form X_i to X_j in the slam algorithm
            In this step the Matrix Omega and the Vector Eta is updated
            The Current position can then later be calculated with z = Omega^-1 * eta
        """
        #indices in the matrix
        x_i = len(self.omega[0]) - 2*self.num_landmarks - 2
        y_i = x_i + 1
        self.extend_matrices()
        x_j = y_i + 1
        y_j = x_j + 1

        #movement in this step
        dx = self.veh.pos[0]
        dy = self.veh.pos[1]

        #exception for zero error, which would give the information a weight of infinite 
        if self.veh.measuring_error == 0:
            measurement_factor = 1
        else:
            measurement_factor = 1.0/self.veh.measuring_error
        if self.veh.movement_error == 0:
            movement_factor = 1
        else:
            movement_factor = 1.0/self.veh.movement_error

        #override the factors because they get very instable
        movement_factor = 1
        measurement_factor = 1

        #update with information of movement
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
        
        #update with landmark information
        landmark_zero = len(self.omega[0])-2*self.num_landmarks #position of the first landmark entry
        tempStateVec = []
        for landmark in self.veh.get_detected():
            l_num = landmark[2] #number of landmark
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
            l_dist = sqrt(pow(l_meas_x,2)+pow(l_meas_y,2))
            phi = self.veh.pi2pi(arctan2(l_meas_y,l_meas_x))
            tempStateVec.append([l_dist, phi-self.veh.phi, phi, l_num])
        self.landmark_array.append(tempStateVec)

        return self.get_estimate()
        # return self.get_estimate() #evaluate the matrices an return the current estimated positon

    def get_estimate(self):
        """Evaluates the linear system and calculates the current position

        Returns:
            List(float): current estimated position
        """
        #the matrix might have landmarks in it that never got detected, making it singular
        remove_count = 0
        #copy the matrices so that the original is not modified
        om = np.copy(self.omega)
        et = np.copy(self.eta)
        undetected = []

        step_num = (len(self.omega[0])-2*self.num_landmarks)/2
        landmark_with_steps = []
        detected_lm_id = step_num
        #check which landmarks were never detected and record the steps realted to each detected landdmarks
        for i in range(len(om)):
            if(om[i][i] == 0):
                undetected.append(i)
        ## added by Bian
            else:
                if i >= (step_num*2) and i%2 == 0:
                    landmark_with_steps.append(
                        {
                            'detected_lm_id':int(detected_lm_id),
                            'related_step': [int(j/2) for j in range(0, len(om[i]), 2) if om[i][j] != 0 and j < step_num*2]
                        }
                    )
                    detected_lm_id += 1

        self.landmark_with_steps = landmark_with_steps
        ###

        #remove undetected landmarks
        for i in reversed(undetected):
            om = np.delete(om,i,0)
            om = np.delete(om,i,1)
            et = np.delete(et,i,0)
            remove_count += 1
        #now that all undetected landmarks are removed, omega is not singular and the linear system can be evaluated
        z = np.matmul(np.linalg.inv(om), et)
        self.z = np.copy(z)
        #z includes all robot positions and all landmark positions
        pos = len(om) - floor(2*(self.num_landmarks-(remove_count/2))) - 2  #get index for newest robot position
        self.pos_marker = pos
        est_pos = [z[pos],z[pos+1]] #get newest positon
        self.veh.path =  np.reshape(np.copy(z[:pos+2]),(-1,2)) #add the estimate to the vehicle
        print(z)
        return [*est_pos] #return the estimate

    def get_error(self):
        """This function calculates the error in the estimation using APE
            TODO Ape calculation
        """
        file_path = "../../Graph-based-SLAM-tutorial-master"
        if(not os.path.exists(file_path)):
            os.mkdir(file_path)
        truthFile = open(file_path+"/truth.txt","w")
        estFile = open(file_path+"/est.txt","w")
        landmarkFile = open(file_path+"/landmarks.txt","w")
        for i in range(len(self.veh.true_path)):
            if(i == 0):
                truthFile.write(str(self.veh.true_path[i][0])+" "+str(self.veh.true_path[i][1])+" 0\n")
            else:
                truthFile.write(str(self.veh.true_path[i][0])+" "+str(self.veh.true_path[i][1])+" "+str(arctan2(self.veh.true_path[i][1]-self.veh.true_path[i-1][1],self.veh.true_path[i][0]-self.veh.true_path[i-1][0]))+"\n")
        for i in range(len(self.veh.path)):
            if(i == 0):
                estFile.write(str(self.veh.path[i][0])+" "+str(self.veh.path[i][1])+" 0\n")
            else:
                estFile.write(str(self.veh.path[i][0])+" "+str(self.veh.path[i][1])+" "+str(arctan2(self.veh.path[i][1]-self.veh.path[i-1][1],self.veh.path[i][0]-self.veh.path[i-1][0]))+"\n")
        for i in range(self.pos_marker+2,len(self.z),2):
            landmarkFile.write(str(self.z[i])+" "+str(self.z[i+1])+" 0\n")
        truthFile.close()
        estFile.close()
        landmarkFile.close()