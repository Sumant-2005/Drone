import numpy as np
from dronesim import params as P
from dronesim.design import margins
from flight_checks import check,save,scenario,step_response


def test_rate_control():
    cases,design = [],[]
    for i,key in enumerate(('p','q','r')):
        wc,pm = margins(i)
        assert abs(wc/P.rate_wc[i]-1)<=.2 and pm>=45
        print(f"{key} BW/PM: {P.rate_wc[i]} +/-20%, >=45 deg | {wc:.5f}, {pm:.5f} | PASS")
        design.append(dict(axis=key,crossover=wc,phase_margin_deg=pm))
        v = np.eye(3)[i]
        c,h = scenario('rate_'+key,10,mode='rate',value=(.05*v).tolist())
        step_response(c,h,key,.05,wc,100,.0005)
        cases.append(c)
        c,h = scenario('release_'+key,6,mode='rate',value=(2*v).tolist(),pulse_end=.075)
        check(c,h,f'axis{i+1}','peak',P.axis_limit[i]-1e-10,P.axis_limit[i]+1e-10,1e-5)
        check(c,h,f'integral{i+1}','peak',0,P.rate_i_limit[i]+1e-10,1e-3)
        peak = check(c,h,key,'max',1e-3,2,.03)
        check(c,h,key,'min',-.30*peak,1e-6,.01,start=.075)
        check(c,h,key,'end',-.002,.002,.001)
        cases.append(c)
    save(3,cases,design=design)
