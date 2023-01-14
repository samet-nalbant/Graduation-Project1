import numpy as np
from enum import Enum
class Position:
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        
class TrajectoryDirection(Enum):
    X = 1
    Y = 2    
TrajectoryDirection = Enum('TrajectoryDirector', ['X', 'Y'])

def eightTrajectoryGenerator(amplitude, frequency, timeFromTrajectoryStart, dronePosition):
    theta = 2 * np.pi * frequency * timeFromTrajectoryStart
    dronePosition.x = amplitude * np.sin(theta)
    dronePosition.y = amplitude * np.cos(theta) * np.sin(theta)
    #position.z = amplitude * (1 - np.cos(2 * theta))
    dronePosition.yaw = 0
    #print(np.mod(theta, 2 * np.pi))
    if round(np.mod(theta, 2 * np.pi),3) == 0.000:
        return True
    else:
        return False