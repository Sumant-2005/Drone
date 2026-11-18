"""Sampled cascade. The integral state is expressed in controller output units."""
import numpy as np
from . import params as P


def pi(error, integral, kp, ki, limit, i_limit):
    raw = kp*error + integral
    output = np.clip(raw,-limit,limit)
    increment = np.where((raw-output)*error>0,0,ki*P.drive_sample_time*error)
    return output,np.clip(integral+increment,-i_limit,i_limit)


def tilt(accel,yaw):
    c,s = np.cos(yaw),np.sin(yaw)
    n,e = accel
    angles = np.array([-s*n+c*e,-(c*n+s*e)])/P.gravity
    return angles/max(1,np.linalg.norm(angles)/P.max_tilt)


class Controller:
    def __init__(self,mass,compensation):
        self.hover = mass*P.gravity/(4*P.max_thrust)
        self.compensation = compensation
        self.rate_i = np.zeros(3)
        self.horizontal_i = np.zeros(2)
        self.climb_i = 0.
        self.height = -P.initial_state[2]

    def velocity(self,command,x,offset=0.):
        accel,self.horizontal_i = pi(command[:2]-x[3:5],self.horizontal_i,
            P.velocity_kp,P.velocity_ki,P.accel_limit,P.accel_limit)
        angles = tilt(accel,x[8])
        climb = (1-P.altitude_kp/P.climb_wc)*command[2]+P.altitude_kp*(self.height+offset+x[2])
        self.height += P.drive_sample_time*command[2]
        inc,self.climb_i = pi(climb+x[5],self.climb_i,P.climb_kp,P.climb_ki,P.climb_limit,P.climb_limit)
        collective = self.hover+inc
        if self.compensation:
            collective /= max(np.cos(x[6])*np.cos(x[7]),np.cos(P.max_tilt)**2)
        return np.r_[angles,command[3]],np.clip(collective,0,1)

    def attitude(self,command,x):
        return np.r_[np.clip(P.attitude_kp*(command[:2]-x[6:8]),-P.angle_rate_limit,P.angle_rate_limit),command[2]]

    def rate(self,command,x):
        axis,self.rate_i = pi(command-x[9:12],self.rate_i,P.rate_kp,P.rate_ki,P.axis_limit,P.rate_i_limit)
        return axis
