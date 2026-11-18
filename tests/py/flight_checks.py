"""Common numerical cases and comparison records for the MATLAB benches."""
import json
from functools import lru_cache
from pathlib import Path
import numpy as np
from dronesim import params as P
from dronesim.sim import flight
from dronesim.body import STATE_NAMES

ROOT = Path(__file__).resolve().parents[2]


@lru_cache(maxsize=100)
def _run(key):
    options = json.loads(key)
    for name in ("value","table"):
        if isinstance(options.get(name),list):
            options[name] = np.asarray(options[name])
    return flight(**options)


def scenario(name,duration,**kwargs):
    inputs = {"duration":duration,**kwargs}
    return {"name":name,"inputs":inputs,"checks":[]},_run(json.dumps(inputs,sort_keys=True))


def signals(h):
    x,speed,torque = h[:,1:13],h[:,13:17],h[:,21:25]
    out = dict(zip(STATE_NAMES,x.T))
    out.update(height=-x[:,2],climb=-x[:,5],altitude_change=-x[:,2]+P.initial_state[2],
               position_error=np.linalg.norm(x[:,:3]-P.initial_state[:3],axis=1),
               horizontal_speed=np.linalg.norm(x[:,3:5],axis=1),
               tilt=np.linalg.norm(h[:,29:31],axis=1),total_kw=np.sum(speed*torque,axis=1)/1000)
    for i in range(4):
        out[f"rpm{i+1}"] = speed[:,i]*30/np.pi
        out[f"torque{i+1}"] = torque[:,i]
        out[f"kw{i+1}"] = speed[:,i]*torque[:,i]/1000
    for i in range(3):
        out[f"axis{i+1}"] = h[:,35+i]
        out[f"integral{i+1}"] = h[:,42+i]
    return out


def measured(t,y,op,target):
    if op=="end": return y[-1]
    if op=="max": return y.max()
    if op=="min": return y.min()
    if op=="peak": return abs(y).max()
    if op=="change": return y[-1]-y[0]
    if op=="overshoot": return max(0.,100*(np.max(y/target)-1))
    def crossing(level):
        z = y/target
        i = np.flatnonzero(z>=level)[0]
        return t[i] if i==0 else np.interp(level,z[i-1:i+1],t[i-1:i+1])
    if op=="rise": return crossing(.9)-crossing(.1)
    if op=="tau": return crossing(1-np.exp(-1))
    raise ValueError(op)


def check(case,h,signal,op,low,high,atol,*,target=1.,start=0.,stop=None,rel=0.):
    stop = h[-1,0] if stop is None else stop
    select = (h[:,0]>=start-1e-10)&(h[:,0]<=stop+1e-10)
    value = float(measured(h[select,0],signals(h)[signal][select],op,target))
    passed = low<=value<=high
    print(f"{case['name']}/{signal}/{op}: [{low:g}, {high:g}] | {value:.9g} | {'PASS' if passed else 'FAIL'}")
    assert passed
    case["checks"].append(dict(signal=signal,op=op,start=start,stop=stop,target=target,
                               value=value,atol=atol,rtol=rel,low=low,high=high))
    return value


def save(stage,cases,**extra):
    (ROOT/'expected'/f'stage{stage}.json').write_text(json.dumps(dict(cases=cases,**extra),
        indent=2,allow_nan=False)+'\n')


def step_response(c,h,key,target,wc,overshoot,steady):
    check(c,h,key,'rise',.01,8/wc,.002,target=target,rel=.15)
    check(c,h,key,'tau',.15/wc,3.5/wc,.005,target=target,rel=.15)
    check(c,h,key,'overshoot',0,overshoot,5.,target=target)
    check(c,h,key,'end',target-steady,target+steady,steady)
