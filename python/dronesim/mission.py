"""Look up the active maneuver and rotate body velocity into NED."""
import numpy as np
from . import params as P


def command(t, yaw, table=None):
    rows = P.mission_table if table is None else table
    i = max(0, np.searchsorted(rows[:,0],t,side="right")-1)
    forward,right,up,yaw_deg = rows[i,1:]
    c,s = np.cos(yaw),np.sin(yaw)
    return np.array([c*forward-s*right,s*forward+c*right,up,np.deg2rad(yaw_deg)])
