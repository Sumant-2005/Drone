"""Independent SI formulas and kinematic quadrature. Uses no dronesim imports.

Run after pytest from the repository root. Position is reconstructed by integrating
velocity. The yaw angle is reconstructed from Euler kinematics. Hover speed, load
torque and power come from mass balance. The mirror test uses measured rotor records.
"""
import json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid


def recheck(root):
    folder = root/'build'/'flight-recheck'
    fixture = json.loads((root/'expected'/'stage8.json').read_text())
    records = {}
    failures = []
    for case in fixture['cases']:
        name = case['name']
        with np.load(folder/(name+'.npz')) as z:
            t,v,angles,rates,w = (z[k] for k in ('time','velocity','angles','rates','speed'))
        records[name] = w
        delta = cumulative_trapezoid(v,t,axis=0,initial=0)
        phi,theta = angles[:,:2].T
        yaw_rate = (np.sin(phi)*rates[:,1]+np.cos(phi)*rates[:,2])/np.cos(theta)
        yaw = cumulative_trapezoid(yaw_rate,t,initial=0)
        columns = dict(north=delta[:,0],east=delta[:,1],altitude_change=-delta[:,2],
                       height=5-delta[:,2],position_error=np.linalg.norm(delta,axis=1),yaw=yaw,
                       v_north=v[:,0],v_east=v[:,1],climb=-v[:,2],r=rates[:,2])
        mass = case['inputs'].get('mass',300)
        omega = np.sqrt(mass*9.81/(4*.01293))
        torque = .0009*omega**2
        for i in range(1,5):
            for k,x in [('rpm',30*omega/np.pi),('torque',torque),('kw',torque*omega/1000)]:
                columns[k+str(i)] = np.full_like(t,x)
        columns['total_kw'] = np.full_like(t,4*torque*omega/1000)
        for metric in case['checks']:
            y = columns[metric['signal']]
            op = metric['op']
            got = {'peak':lambda:np.max(abs(y)),'end':lambda:y[-1],'min':lambda:y.min()}[op]()
            expected = metric['value']
            passed = abs(got-expected)<max(1e-6,2e-4 if op=='min' or metric['signal']=='yaw' else 1e-5)
            print(f"recheck {name}/{metric['signal']}: {expected:.9g} | {got:.9g} | {'PASS' if passed else 'FAIL'}")
            if not passed: failures.append((name,metric['signal'],expected,got))
    for m in fixture['mirrors']:
        a,b = (records[m[k]]-np.sqrt(300*9.81/(4*.01293)) for k in ('first','second'))
        perm = [2,3,0,1] if m['axis']==3 else [1,0,3,2]
        mismatch = abs(a+b if m['axis']==2 else a-b[:,perm]).max()/abs(a).max()
        if abs(mismatch-m['relative_error'])>1e-9: failures.append((m['first'],'mirror',mismatch))
    if failures: raise AssertionError(f"Independent disagreements: {failures}")
    print('Independent recheck: no disagreements outside the stated integration tolerance.')


if __name__=='__main__':
    recheck(Path(__file__).resolve().parents[2])
