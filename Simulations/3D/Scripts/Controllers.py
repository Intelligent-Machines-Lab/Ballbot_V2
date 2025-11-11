import time
import numpy as np

class LQR:
    def __init__(self, lqr_gains):
        self.k1, self.k2, self.k3, self.k4 = lqr_gains
        
    def update(self, erro):
        # X --> [x, x_dot, theta, theta_dot, y, y_dot, phi, phi_dot]

        erro[0] = Saturate(erro[0], 0.25)
        erro[4] = Saturate(erro[4], 0.25)
        
        Tx = erro[4]*self.k1 + erro[5]*self.k2 + erro[6]*self.k3*1.750 + erro[7]*self.k4*1.750
        Ty = -erro[0]*self.k1 - erro[1]*self.k2 + erro[2]*self.k3 + erro[3]*self.k4
        
        return np.round(Tx, 4), np.round(Ty, 4)

class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.dt = dt

    def update(self, target, measurement):      
        error = target - measurement

        P = self.kp * error

        self.integral += error * self.dt
        I = self.ki * self.integral

        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0
        D = self.kd * derivative

        output = P + I + D

        self.prev_error = error

        return output
        
def Saturate(value, sat):
    if value >= sat:
        value = sat
        
    elif value <= -sat:
        value = -sat
        
    return value