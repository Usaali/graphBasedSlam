import random as rand
import world

class vehicle:

    def __init__(self, plane: world.world,  movementError: float = 1, measuringError: float = 1, sensRange: float = 50):
        """A vehicle object, that can move and sense in a world

        Args:
            plane (world.world): The world to spawn the vehicle in
            movementError (float, optional): The error that is added on any movement. Defaults to 1.
            measuringError (float, optional): The error that is added on any measurement. Defaults to 1.
            sensRange (float, optional): The range of the vehicle sensors. Defaults to 50.
        """
        self.world = plane
        self.plane_size = plane.size
        self.true_pos = [self.plane_size/2,self.plane_size/2]
        self.pos = [self.plane_size/2,self.plane_size/2]
        self.sense_range = sensRange
        self.movement_error = movementError
        self.measuring_error = measuringError
        self.detected_landmarks = []
        self.path = []
        self.true_path = []
    
    def rand(self, mean, std):
        r = rand.gauss(mean,std)
        return rand.gauss(mean,std)

    def get_detected(self):
        return self.detected_landmarks

    def move(self,dx,dy):
        x = self.rand(self.pos[0] + dx, self.movement_error)
        y = self.rand(self.pos[1] + dy, self.movement_error)
        if x < 0.0 or x > self.plane_size or y < 0.0 or y > self.plane_size:
            return False
        else:
            self.true_pos[0] += dx
            self.true_pos[1] += dy
            self.pos[0] = x
            self.pos[1] = y
            self.path.append([x,y])
            self.true_path.append([*self.true_pos])
            return True
    
    def get_manhatten(self, pos1):
        return abs(self.true_pos[0]-pos1[0]) + abs(self.true_pos[1] - pos1[1])


    def sense(self):
        self.detected_landmarks = []
        for l in self.world.getLandmarks():
            if self.get_manhatten(l) <= self.sense_range :
                self.detected_landmarks.append([self.rand(l[0],self.measuring_error),self.rand(l[1],self.measuring_error)])
    