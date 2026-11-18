import numpy as np
from dronesim import params as P
from flight_checks import ROOT,check,save,scenario
from recheck_stage8 import recheck


def test_system():
    cases,traces,mirrors = [],{},[]
    for scale in (1.,1.2):
        c,h = scenario('hover' if scale==1 else 'heavy_hover',5,mode='velocity',value=[0,0,0,0],mass=P.drone_mass*scale)
        speed = np.sqrt(P.drone_mass*scale*P.gravity/(4*P.kT_prop))
        torque = P.kQ_prop*speed**2
        check(c,h,'position_error','peak',0,1e-7,1e-7)
        for i in range(1,5):
            for key,target in [('rpm',speed*30/np.pi),('torque',torque),('kw',torque*speed/1000)]:
                check(c,h,key+str(i),'end',target-1e-6,target+1e-6,1e-5)
        check(c,h,'total_kw','end',4*torque*speed/1000-1e-6,4*torque*speed/1000+1e-6,1e-5)
        cases.append(c); traces[c['name']] = h
    for i,name in enumerate(('forward','back','right','left','up','down','yaw_right','yaw_left')):
        start,end = P.mission_table[2*i:2*i+2,0]
        duration = end-start
        move = P.mission_table[2*i,1:]
        table = [[0,*move],[duration,0,0,0,0],[duration+3,0,0,0,0]]
        c,h = scenario(name,duration+3,table=table)
        idx = i//2
        channel = ('north','east','altitude_change','yaw')[idx]
        rate = move[idx]*(np.pi/180 if idx==3 else 1.)
        target = rate*duration
        allowance = abs(rate)*.5
        check(c,h,channel,'end',target-allowance,target+allowance,.015)
        velocity = ('v_north','v_east','climb','r')[idx]
        check(c,h,velocity,'end',-abs(rate)*.1,abs(rate)*.1,.005)
        cases.append(c); traces[name] = h
        if i%2:
            previous = cases[-2]['name']
            a = traces[previous][:,13:17]-P.hover_speed
            b = h[:,13:17]-P.hover_speed
            if idx==2:
                residual = a+b
            else:
                permutation = [2,3,0,1] if idx==3 else [1,0,3,2]
                residual = a-b[:,permutation]
            mismatch = float(np.max(abs(residual))/np.max(abs(a)))
            print(f"{name} mirror error: <=0.15 | {mismatch:.6g} | {'PASS' if mismatch<=.15 else 'FAIL'}")
            assert mismatch<=.15
            mirrors.append(dict(first=previous,second=name,axis=idx,relative_error=mismatch,limit=.15,atol=.01))
    c,h = scenario('full_mission',float(P.mission_table[-1,0]))
    check(c,h,'position_error','end',0,.5,.015)
    check(c,h,'height','min',2.,5.000001,.01)
    cases.append(c); traces[c['name']] = h
    losses = []
    for enabled in (False,True):
        c,h = scenario('forward_'+str(int(enabled)),10,mode='velocity',value=[2,0,0,0],compensation=enabled)
        losses.append(-check(c,h,'altitude_change','min',-.5,1e-8,.0005))
        cases.append(c); traces[c['name']] = h
    assert losses[1] < losses[0]
    save(8,cases,mirrors=mirrors)
    recheck(ROOT)
