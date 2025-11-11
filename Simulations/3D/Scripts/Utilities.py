import time
import numpy as np

class LowPassFilter():
    def __init__(self, alpha=0.05):
        self.filtered_value = 0
        self.initialized = False
        self.alpha = alpha
    
    def update(self, raw_value):
        if not self.initialized:
            self.filtered_value = raw_value
            self.initialized = True
            
        else:
            self.filtered_value = raw_value*self.alpha + (1-self.alpha)*self.filtered_value

        return self.filtered_value

def RotationMatrix(axis, angle):
    if axis == 1:
        R = np.array([[1,              0,             0],
                      [0,  np.cos(angle), np.sin(angle)],
                      [0, -np.sin(angle), np.cos(angle)]])
        return np.round(R, 8)
    
    elif axis == 2:
        R = np.array([[np.cos(angle), 0, -np.sin(angle)],
                      [            0, 1,              0],
                      [np.sin(angle), 0,  np.cos(angle)]])
        return np.round(R, 8)

    else:
        R = np.array([[ np.cos(angle), np.sin(angle), 0],
                      [-np.sin(angle), np.cos(angle), 0],
                      [             0,             0, 1]])
        return np.round(R, 8)
        
def UpdatePosition(self, x_w, y_w):
    tolerance = 0.05 # 5 cm radius
    self.x_w = x_w
    self.y_w = y_w
    
    x_target, y_target = self.waypoints[self.currentWaypoint]
    distance = np.sqrt((self.x_w - x_target)**2 + (self.y_w - y_target)**2)
    
    if distance <= tolerance:
        if self.currentWaypoint < len(self.waypoints) - 1:
            self.currentWaypoint += 1
            self.xPosRef = self.waypoints[self.currentWaypoint][0]
            self.yPosRef = self.waypoints[self.currentWaypoint][1]
            print(f"Moving to waypoint {self.currentWaypoint}: ({self.xPosRef}, {self.yPosRef})")
        else:
            print("Final waypoint reached.")
            