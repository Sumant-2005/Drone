from flight_checks import scenario,check,save


def test_assembled_hover():
    c,h = scenario('assembled_hover',5,mode='velocity',value=[0,0,0,0])
    check(c,h,'position_error','peak',0,1e-7,1e-7)
    save(7,[c])
