import numpy as np
from dronesim import params as P
from dronesim.mission import command
from flight_checks import save


def test_mission():
    table = P.mission_table
    assert table.shape==(17,5) and table[0,0]==0 and table[-1,0]<60
    durations = np.diff(table[:,0])
    assert np.all(durations[1::2]>=3) and np.all(table[1::2,1:]==0)
    legs = table[:-1:2,1:]
    assert np.all(np.count_nonzero(legs,axis=1)==1)
    np.testing.assert_array_equal(legs[0::2],-legs[1::2])
    np.testing.assert_array_equal(durations[0::4],durations[2::4])
    np.testing.assert_array_equal(legs[[0,2,4,6]],[ [2,0,0,0],[0,2,0,0],[0,0,1,0],[0,0,0,10] ])
    expected = [8,-8,8,-8,3,-3,30,-30]
    np.testing.assert_array_equal(legs.sum(axis=1)*durations[::2],expected)
    samples = []
    for t in sorted(set(table[:,0])|{1.,9.,20.,30.,47.}):
        for yaw in (0.,np.pi/2):
            got = command(t,yaw)
            row = table[table[:,0]<=t][-1,1:]
            target = [row[0],row[1],row[2],np.deg2rad(row[3])] if yaw==0 else [-row[1],row[0],row[2],np.deg2rad(row[3])]
            np.testing.assert_allclose(got,target,atol=1e-12)
            samples.append(dict(t=t,yaw=yaw,value=got.tolist(),atol=1e-10))
    print(f"mission duration and leg integrals: 52 s, {expected} | {table[-1,0]} s, {expected} | PASS")
    save(6,[],samples=samples)
