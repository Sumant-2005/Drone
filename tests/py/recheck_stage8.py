"""Independent flight recheck from the design constants and experiment inputs.

No dronesim imports, saved trajectories, or controller gains from the primary model.
Integrate the body with midpoint quadrature and the ideal motor with its analytic
exponential solution between successive reference samples. Compare every S8 metric.
"""
import json
from math import cos, exp, hypot, pi, sin, sqrt
from pathlib import Path

import numpy as np


def limited(x,bound):
    return min(bound,max(-bound,x))


def regulator(error,integrals,index,kp,ki,cap,icap,dt):
    raw = kp*error+integrals[index]
    out = limited(raw,cap)
    if (raw-out)*error<=0:
        integrals[index]=limited(integrals[index]+ki*error*dt,icap)
    return out


def standard_table():
    rows=[]
    time=0.
    for axis in range(4):
        for sign in (1,-1):
            demand=[0.]*4
            demand[axis]=sign*(2,2,1,10)[axis]
            rows.append([time,*demand])
            time+=(4,4,3,3)[axis]
            rows.append([time,0,0,0,0])
            time+=3
    return rows+[[time,0,0,0,0]]


def replay(options):
    mass=options.get('mass',300.)
    dt=.00125
    tau=.0083
    g=9.81
    kt,kq=.01293,.0009
    j=.7+.0383
    wmax=100*pi
    max_thrust=kt*wmax*wmax
    nominal=sqrt(300*g/(4*kt))
    trim=sqrt(mass*g/(4*kt))
    slew=.8*((1.5*10*.0542)*(340/1.5)-kq*wmax*wmax)/j
    wc=8.
    kr=4*1.1*max_thrust/100
    kp_r=wc*hypot(1,tau*wc)/(kr*hypot(1,.25))
    gy=4*wmax*wmax/(2*nominal)*abs(2*kq*nominal+2j*j)/(abs(50+2j*185)*hypot(1,2*tau))
    ki_y=2*sqrt(1/gy**2-.05**2)
    kvz=2.5*hypot(1,2.5*tau)/(4*max_thrust/300)
    tilt_bound=pi/9
    table=options.get('table',standard_table())
    integrals=[0.]*6
    height=5.
    x=np.zeros(12); x[2]=-5.
    w=np.full(4,trim)
    ref=w.copy()
    end=round(options['duration']/dt)
    rows=[]
    leg=0

    def derivative(x,speeds,accels):
        phi,theta,psi,p,q,r=x[6:]
        cf,sf,ct,st,cy,sy=cos(phi),sin(phi),cos(theta),sin(theta),cos(psi),sin(psi)
        a,b,c,d=kt*speeds**2
        thrust=(a+b+c+d)/mass
        shaft=kq*speeds*abs(speeds)+j*accels
        result=np.empty(12)
        result[:3]=x[3:6]
        result[3:6]=(-thrust*(cf*st*cy+sf*sy)-20*x[3]/mass,
                     -thrust*(cf*st*sy-sf*cy)-20*x[4]/mass,
                     g-thrust*cf*ct-20*x[5]/mass)
        result[6:9]=(p+st*(sf*q+cf*r)/ct,cf*q-sf*r,(sf*q+cf*r)/ct)
        result[9:]=((1.1*(-a+b+c-d)-85*q*r-50*p)/100,
                    (1.1*(a-b+c-d)+85*p*r-50*q)/100,
                    (shaft[0]+shaft[1]-shaft[2]-shaft[3]-50*r)/185)
        return result

    for k in range(end+1):
        t=k*dt
        if options.get('mode','mission')=='velocity':
            n,e,up,yaw_rate=options['value']
        else:
            while leg+1<len(table) and t>=table[leg+1][0]: leg+=1
            forward,right,up,deg=table[leg][1:]
            n=cos(x[8])*forward-sin(x[8])*right
            e=sin(x[8])*forward+cos(x[8])*right
            yaw_rate=deg*pi/180
        an=regulator(n-x[3],integrals,0,.65,.65/15,g*tilt_bound,g*tilt_bound,dt)
        ae=regulator(e-x[4],integrals,1,.65,.65/15,g*tilt_bound,g*tilt_bound,dt)
        roll=(-sin(x[8])*an+cos(x[8])*ae)/g
        pitch=-(cos(x[8])*an+sin(x[8])*ae)/g
        factor=max(1,hypot(roll,pitch)/tilt_bound)
        climb=.75*up+.625*(height+x[2])
        height+=dt*up
        collective=mass*g/(4*max_thrust)+regulator(climb+x[5],integrals,2,kvz,kvz/15,.35,.35,dt)
        if options.get('compensation',True):
            collective/=max(cos(x[6])*cos(x[7]),cos(tilt_bound)**2)
        collective=min(1,max(0,collective))
        roll_rate=limited(2*(roll/factor-x[6]),.7)
        pitch_rate=limited(2*(pitch/factor-x[7]),.7)
        a=regulator(roll_rate-x[9],integrals,3,kp_r,2*kp_r,.15,.1,dt)
        b=regulator(pitch_rate-x[10],integrals,4,kp_r,2*kp_r,.15,.1,dt)
        c=regulator(yaw_rate-x[11],integrals,5,.05,ki_y,.08,.08,dt)
        mix=np.array([-a+b+c,a-b+c,a+b-c,-a-b-c])+collective
        target=wmax*np.sqrt(np.clip(mix,0,1))
        ref+=np.clip(target-ref,-dt*slew,dt*slew)
        if k%4==0 or k==end:
            rows.append(np.r_[x,w,kq*w*w+j*(ref-w)/tau])
        if k<end:
            # Two midpoint substeps use exact motor speeds at each quadrature node.
            for _ in range(2):
                h=dt/2
                mid_w=ref+(w-ref)*exp(-h/(2*tau))
                mid_x=x+.5*h*derivative(x,w,(ref-w)/tau)
                x+=h*derivative(mid_x,mid_w,(ref-mid_w)/tau)
                w=ref+(w-ref)*exp(-h/tau)
                assert x[2]<0, 'S8 airborne recheck encountered ground contact'
    return np.array(rows)


def recheck(root):
    fixture=json.loads((root/'expected'/'stage8.json').read_text())
    failures=[]
    speeds={}
    for case in fixture['cases']:
        name=case['name']
        a=replay(case['inputs'])
        w=a[:,12:16]; torque=a[:,16:20]; speeds[name]=w
        delta=a[:,:3]-[0,0,-5]
        columns=dict(north=a[:,0],east=a[:,1],height=-a[:,2],altitude_change=-delta[:,2],
                     position_error=np.linalg.norm(delta,axis=1),v_north=a[:,3],v_east=a[:,4],
                     climb=-a[:,5],yaw=a[:,8],r=a[:,11],total_kw=np.sum(w*torque,axis=1)/1000)
        for i in range(4):
            columns[f'rpm{i+1}']=30*w[:,i]/pi
            columns[f'torque{i+1}']=torque[:,i]
            columns[f'kw{i+1}']=w[:,i]*torque[:,i]/1000
        for m in case['checks']:
            y=columns[m['signal']]
            got={'peak':lambda:max(abs(y)),'end':lambda:y[-1],'min':lambda:min(y)}[m['op']]()
            # A different integrator agrees to 0.2 mm on airborne displacements.
            tolerance=1e-6 if 'hover' in name else .0002
            ok=abs(got-m['value'])<=tolerance
            print(f"independent {name}/{m['signal']}: {m['value']:.9g} | {got:.9g} | {'PASS' if ok else 'FAIL'}")
            if not ok: failures.append((name,m['signal'],got,m['value']))
    for m in fixture['mirrors']:
        a,b=(speeds[m[k]]-sqrt(300*9.81/(4*.01293)) for k in ('first','second'))
        perm=[2,3,0,1] if m['axis']==3 else [1,0,3,2]
        got=abs(a+b if m['axis']==2 else a-b[:,perm]).max()/abs(a).max()
        if abs(got-m['relative_error'])>.0002: failures.append((m['first'],'mirror',got))
    if failures: raise AssertionError(f"Independent disagreements: {failures}")
    print('Independent closed-loop replay: all S8 comparisons passed.')


if __name__=='__main__':
    recheck(Path(__file__).resolve().parents[2])
