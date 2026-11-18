"""Hover small-signal response with motor lag, drag, and shaft acceleration torque."""
import numpy as np
from . import params as P


def margins(axis, attitude=False):
    freq = np.geomspace(.01,1e4,24000)
    s = 1j*freq
    if axis<2:
        inertia = (P.inertia_xx,P.inertia_yy)[axis]
        plant = P.rate_plant_gain[axis]/((s+P.drag_rotational/inertia)*(1+s*P.motor_time_constant))
    else:
        plant = (4*P.speed_per_command*(P.J_total*s+2*P.kQ_prop*P.hover_speed)
                 /((P.inertia_zz*s+P.drag_rotational)*(1+s*P.motor_time_constant)))
    loop = plant*(P.rate_kp[axis]+P.rate_ki[axis]/s)
    if attitude:
        loop = P.attitude_kp*loop/(1+loop)/s
    loggain = np.log(abs(loop))
    hits = np.flatnonzero((loggain[:-1]>=0)&(loggain[1:]<0))
    assert len(hits)==1
    i = hits[0]
    a = loggain[i]/(loggain[i]-loggain[i+1])
    wc = np.exp((1-a)*np.log(freq[i])+a*np.log(freq[i+1]))
    phase = np.rad2deg(np.unwrap(np.angle(loop)))
    return wc,180+(1-a)*phase[i]+a*phase[i+1]
