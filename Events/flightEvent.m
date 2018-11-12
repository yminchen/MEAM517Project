%%
function [position, isterminal, direction] = flightEvent(t,x,phase)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    if phase == 1
        Frontfoot = posFootR(x,sysParam);
        Rearfoot = posFootL(x,sysParam);
    elseif phase == 4
        Frontfoot = posFootL(x,sysParam);
        Rearfoot = posFootR(x,sysParam);
    end
    kneeR = posKneeR(x,sysParam);
    hip = posHip(x,sysParam);
    kneeL = posKneeL(x,sysParam);
    
    position(1)     = Frontfoot(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
    position(2)     = kneeR(2);
    isterminal(2)   = 1;
    direction(2)    = -1;
    position(3)     = hip(2);
    isterminal(3)   = 1;
    direction(3)    = -1;
    position(4)     = x(2);
    isterminal(4)   = 1;
    direction(4)    = -1;
    position(5)     = Rearfoot(2);
    isterminal(5)   = 1;
    direction(5)    = -1;
    position(6)     = kneeL(2);
    isterminal(6)   = 1;
    direction(6)    = -1;
end