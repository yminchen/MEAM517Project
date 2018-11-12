function qplus = impactVelUpdate(x,phase)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    n = size(x,1);
    M = MassMatrix(x(1:n/2),sysParam);
    if phase == 2
        J = JcontPointR(x(1:n/2),sysParam);
    elseif phase == 5
        J = JcontPointL(x(1:n/2),sysParam);
    end
    qplus = (eye(n/2) - (M\J')*pinv(J*(M\J'))*J)*x(n/2+1:n);
end