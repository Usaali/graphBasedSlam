# graphBasedSlam
This is an implementation of a Graph Based Slam for a University Seminar written by Zituo Bian and Usama Ali
It contains Python code to simulate a robot moving around and detecting randomly generated landmarks.
The frontend of our SLAM is written in Python and the Backend is written in Matlab.

## Requirements
- Python 3 (Python 2 might also work but is not tested)
    - numpy
    - matplotlib
    - seaborn
    - pandas
- Matlab

## Usage of the simulation
### Initialization
To simulate the robot, you need to create a world object first. It will automatically create landmarks an place them at random locations. Then you can create a vehicle object an pass the created world as argument. The robot will be placed in the middle of it.
### Visualization
To visualize everything, the plot function of the world object needs to be called. It has to be given the vehicle, that needs to be plotted. The plot will show all landmarks, the current robot position with and without noise, the groundtruth, the noisy trajectory and (if calculated) the trajectory calculated by the frontend.
If QT5 is installed on the system, the plotting backend can be switched in the world.py file 
### Movement
Moving the robot is quite simple. Just call the move function of the vehicle object and pass it the delta x and delta y value it should move. Examples on how to move on a straight line, a circle or in a random direction can be found in run_slam.py
After every movement, the robot automatically senses the landmarks and updates its list. It can be accesses with the get_detected function of the vehicle object.

## Usage of the frontend
To initialize the SLAM frontend, a slam object needs to be created. It needs to be passed the vehicle object and can be given a start position, if it differs from the world center.
After every movement, the slam_step function needs to be called. This function updates the Matrix omega and the vector eta. It also evaluates the linear system and returns the current estimated position. The whole solution is saved in the variable z of the slam object and can be accessed from there if needed. The slam_step function updates all needed variables, so that the estimated trajectorie will be automatically plotted on the next call to plot of the world object.

To use the Matlab optimization, the data needs to be exported as text files into the Backend folder. An example for that can be found in the backendData function in run_slam.py

To make the usage of this program easier, everything needed is already created in run_slam.py. You can change the parameters as needed and use one of the three drive functions to move the robot on a certain trajectory. After that, backendData is called and exports the files directly into the backend folder, to be used with the Matlab script. 

## Usage of the backend
There currently are two ways of using the optimization in Matlab:
### Calculate state vector in matlab
To calculate the state vector, needed for the optimization process in Matlab, the main.m file can be used. There you can adjust a few parameters:
- Ts: The sampling time. Changing this will skip timesteps
- T_gslam: This is the interval, after which an optimization should be done and plotted (does not change the endresult)
- MAX_ITR: This is the amount of iterations, the optimization should try at max. (this can make the result better, but also slows down the process if too high)
- EYESIGHT: This is the sensing range of the robot. Because the state vector is generated in Matlab, it needs to be specified here too
Other than that, everything should stay the same and you can run main.m to optimize and plot the result (this might take a while)

### Calculate the state vector in python (does not work as well)
The state vector is actually also exported into the landmark_vec.txt file, after using the backendData method. To use this, the optimize_python_data.m file can be used. But because this does not give a good result in comparison to the other method, so use it at you own risk. Because it generates less edges, it is much faster as calculating the state vector in Matlab and can propably be made to achieve similar results with a bit more work.