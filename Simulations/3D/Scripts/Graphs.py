import math
import numpy as np

def init_graphs(self, sim):
    self.graph_pos = sim.getObject('/Pos')
    self.stream_pos_x = sim.addGraphStream(self.graph_pos, 'x', 'm', 0, [1, 0, 0])
    self.stream_pos_y = sim.addGraphStream(self.graph_pos, 'y', 'm', 0, [0, 1, 0])

    self.graph_pos_w = sim.getObject('/Pos_W')
    self.stream_pos_w_x = sim.addGraphStream(self.graph_pos_w, 'x_w', 'm', 0, [1, 0, 0])
    self.stream_pos_w_y = sim.addGraphStream(self.graph_pos_w, 'y_w', 'm', 0, [0, 1, 0])

    self.graph_vel = sim.getObject('/Vel')
    self.stream_vel_x = sim.addGraphStream(self.graph_vel, 'vel x', 'm/s', 0, [1, 0, 0])
    self.stream_vel_y = sim.addGraphStream(self.graph_vel, 'vel y', 'm/s', 0, [0, 1, 0])

    self.graph_vel_w = sim.getObject('/Vel_W')
    self.stream_vel_w_x = sim.addGraphStream(self.graph_vel_w, 'vel_w x', 'm/s', 0, [1, 0, 0])
    self.stream_vel_w_y = sim.addGraphStream(self.graph_vel_w, 'vel_w y', 'm/s', 0, [0, 1, 0])

    self.graph_robot_attitude = sim.getObject('/Attitude')
    self.stream_robot_roll = sim.addGraphStream(self.graph_robot_attitude, 'roll', 'deg', 0, [1, 0, 0])
    self.stream_robot_pitch = sim.addGraphStream(self.graph_robot_attitude, 'pitch', 'deg', 0, [0, 1, 0])
    self.stream_robot_yaw = sim.addGraphStream(self.graph_robot_attitude, 'yaw', 'deg', 0, [0, 0, 1])

    self.graph_robot_ang_vel = sim.getObject('/Angular_Velocities')
    self.stream_robot_p = sim.addGraphStream(self.graph_robot_ang_vel, 'p', 'deg/s', 0, [1, 0, 0])
    self.stream_robot_q = sim.addGraphStream(self.graph_robot_ang_vel, 'q', 'deg/s', 0, [0, 1, 0])
    self.stream_robot_r = sim.addGraphStream(self.graph_robot_ang_vel, 'r', 'deg/s', 0, [0, 0, 1])
    
    self.graph_torque_2D = sim.getObject('/Torque_2D')
    self.stream_torque_2D_Tx = sim.addGraphStream(self.graph_torque_2D, 'Tx', 'N.m', 0, [1, 0, 0])
    self.stream_torque_2D_Ty = sim.addGraphStream(self.graph_torque_2D, 'Ty', 'N.m', 0, [0, 1, 0])
    self.stream_torque_2D_Tz = sim.addGraphStream(self.graph_torque_2D, 'Tz', 'N.m', 0, [0, 0, 1])
    
    self.graph_velocity_2D = sim.getObject('/Wheel_Vel_2D')
    self.stream_velocity_2D_Vx = sim.addGraphStream(self.graph_velocity_2D, 'Wx', 'deg/s', 0, [1, 0, 0])
    self.stream_velocity_2D_Vy = sim.addGraphStream(self.graph_velocity_2D, 'Wy', 'deg/s', 0, [0, 1, 0])
    self.stream_velocity_2D_Vz = sim.addGraphStream(self.graph_velocity_2D, 'Wz', 'deg/s', 0, [0, 0, 1])
    
    self.graph_velocity_3D = sim.getObject('/Wheel_Vel_3D')
    self.stream_velocity_3D_V1 = sim.addGraphStream(self.graph_velocity_3D, 'W1', 'deg/s', 0, [1, 0, 0])
    self.stream_velocity_3D_V2 = sim.addGraphStream(self.graph_velocity_3D, 'W2', 'deg/s', 0, [0, 1, 0])
    self.stream_velocity_3D_V3 = sim.addGraphStream(self.graph_velocity_3D, 'W3', 'deg/s', 0, [0, 0, 1])
    
def update_graphs(self, sim):
    sim.setGraphStreamValue(self.graph_pos, self.stream_pos_x, self.x)
    sim.setGraphStreamValue(self.graph_pos, self.stream_pos_y, self.y)
    
    sim.setGraphStreamValue(self.graph_pos_w, self.stream_pos_w_x, self.x_w)
    sim.setGraphStreamValue(self.graph_pos_w, self.stream_pos_w_y, self.y_w)

    sim.setGraphStreamValue(self.graph_vel, self.stream_vel_x, self.u)
    sim.setGraphStreamValue(self.graph_vel, self.stream_vel_y, self.v)

    sim.setGraphStreamValue(self.graph_vel_w, self.stream_vel_w_x, self.u_w)
    sim.setGraphStreamValue(self.graph_vel_w, self.stream_vel_w_y, self.v_w)

    sim.setGraphStreamValue(self.graph_robot_attitude, self.stream_robot_roll, np.rad2deg(self.roll))
    sim.setGraphStreamValue(self.graph_robot_attitude, self.stream_robot_pitch, np.rad2deg(self.pitch))
    sim.setGraphStreamValue(self.graph_robot_attitude, self.stream_robot_yaw, np.rad2deg(self.yaw))

    sim.setGraphStreamValue(self.graph_robot_ang_vel, self.stream_robot_p, self.p)
    sim.setGraphStreamValue(self.graph_robot_ang_vel, self.stream_robot_q, self.q)
    sim.setGraphStreamValue(self.graph_robot_ang_vel, self.stream_robot_r, self.r)
    
    sim.setGraphStreamValue(self.graph_torque_2D, self.stream_torque_2D_Tx, self.Tx)
    sim.setGraphStreamValue(self.graph_torque_2D, self.stream_torque_2D_Ty, self.Ty)
    sim.setGraphStreamValue(self.graph_torque_2D, self.stream_torque_2D_Tz, self.Tz)
    
    sim.setGraphStreamValue(self.graph_velocity_2D, self.stream_velocity_2D_Vx, np.rad2deg(self.Vx))
    sim.setGraphStreamValue(self.graph_velocity_2D, self.stream_velocity_2D_Vy, np.rad2deg(self.Vy))
    sim.setGraphStreamValue(self.graph_velocity_2D, self.stream_velocity_2D_Vz, np.rad2deg(self.Vz))
    
    sim.setGraphStreamValue(self.graph_velocity_3D, self.stream_velocity_3D_V1, np.rad2deg(self.V1))
    sim.setGraphStreamValue(self.graph_velocity_3D, self.stream_velocity_3D_V2, np.rad2deg(self.V2))
    sim.setGraphStreamValue(self.graph_velocity_3D, self.stream_velocity_3D_V3, np.rad2deg(self.V3))