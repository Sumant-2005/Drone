import numpy as np
from dronesim import params as P
from dronesim.control import tilt
from flight_checks import check,save,scenario,step_response


def test_velocity_and_altitude():
    assert tilt([1,0],0)[1]<0 and tilt([0,1],0)[0]>0 and tilt([1,0],np.pi/2)[0]<0
    cases = []
    for i,key in enumerate(('v_north','v_east','climb','altitude_change')):
        mode = 'altitude' if i==3 else 'velocity'
        c,h = scenario('step_'+key,20,mode=mode,value=1. if i==3 else np.eye(4)[i].tolist())
        wc = [P.velocity_wc,P.velocity_wc,P.climb_wc,P.altitude_kp][i]
        step_response(c,h,key,1,wc,10,.003)
        check(c,h,'tilt','peak',0,P.max_tilt+1e-10,.002)
        cases.append(c)
    loss = []
    for comp in (False,True):
        c,h = scenario('forward_'+str(int(comp)),10,mode='velocity',value=[2,0,0,0],compensation=comp)
        loss.append(-check(c,h,'altitude_change','min',-.5,1e-8,.0005))
        cases.append(c)
    assert loss[1]<loss[0]
    save(5,cases,loss_off_m=loss[0],loss_on_m=loss[1])
