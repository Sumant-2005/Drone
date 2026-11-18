import numpy as np
from dronesim import params as P
from dronesim.design import margins
from flight_checks import check,save,scenario,step_response


def test_attitude_control():
    cases,design = [],[]
    for i,key in enumerate(('roll','pitch')):
        amp = np.deg2rad(5)
        wc,pm = margins(i,True)
        assert pm>=45
        design.append(dict(axis=key,crossover=wc,phase_margin_deg=pm))
        c,h = scenario('angle_'+key,8,mode='attitude',value=(amp*np.eye(3)[i]).tolist())
        step_response(c,h,key,amp,wc,10,.0005)
        check(c,h,f'axis{i+1}','peak',0,P.axis_limit[i]-1e-6,.001)
        cases.append(c)
    c,h = scenario('heading_gap',6,mode='attitude',value=[0,0,np.deg2rad(10)],pulse_end=3.)
    drift = check(c,h,'yaw','change',-np.pi,np.pi,.004,start=3)
    cases.append(c)
    save(4,cases,design=design,heading_drift_deg=np.rad2deg(drift))
