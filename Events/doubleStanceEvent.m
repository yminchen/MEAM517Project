function [position, isterminal, direction] = doubleStanceEvent(t,x,phase,k_des,dx_des)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    n = size(x,1);
    
    %% detect lift off
    tau = groundController(x,phase);
    
    % Calculate Ground Reaction Force
    M = MassMatrix(x(1:n/2),sysParam);
    fCG = FCorGrav(x,sysParam);
    J = [JcontPointR(x(1:n/2),sysParam);
         JcontPointL(x(1:n/2),sysParam)];
    dJ = [dJcontPointR(x,sysParam);
          dJcontPointL(x,sysParam)];
    lamda = -(J*(M\J'))\(J*(M\(fCG+tau)) + dJ*x(n/2+1:n));
        
    position(1)     = lamda(2);
    isterminal(1)   = 1;
    direction(1)    = -1;
    
    position(2)     = lamda(4);
    isterminal(2)   = 1;
    direction(2)    = -1;
    
    %% detect robot falls onto the ground
    kneeR = posKneeR(x,sysParam);
    hip = posHip(x,sysParam);
    kneeL = posKneeL(x,sysParam);
    
    position(3)     = kneeR(2);
    isterminal(3)   = 1;
    direction(3)    = -1;
    position(4)     = hip(2);
    isterminal(4)   = 1;
    direction(4)    = -1;
    position(5)     = x(2);
    isterminal(5)   = 1;
    direction(5)    = -1;
    position(6)     = kneeL(2);
    isterminal(6)   = 1;
    direction(6)    = -1;
    
    %% dectect negative contact force
    position(7)     = lamda(2);
    isterminal(7)   = 1;
    direction(7)    = 1;
    
    
end