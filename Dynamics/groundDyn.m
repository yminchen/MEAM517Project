function dxdt = groundDyn(t,x,phase,t_prev_stance,k_des,dx_des)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    dxdt = zeros(size(x));
    n = size(x,1);
    dxdt(1:n/2) = x(n/2+1:n);
    
    % controller
    tau = groundController(x,phase,t_prev_stance,k_des,dx_des);
    
    % Calculate Ground Reaction Force
    M = MassMatrix(x(1:n/2),sysParam);
    fCG = FCorGrav(x,sysParam);
    if phase==2 || phase==3
        J = JcontPointR(x(1:n/2),sysParam);
        dJ = dJcontPointR(x,sysParam);
    elseif phase==5 || phase==6
        J = JcontPointL(x(1:n/2),sysParam);
        dJ = dJcontPointL(x,sysParam);
    end
    lamda = -(J*(M\J'))\(J*(M\(fCG+tau)) + dJ*x(n/2+1:n));

    dxdt(n/2+1:n) = M\(fCG + J'*lamda + tau);
%     if lamda(2)<0
%         dxdt(n/2+1:n) = M\(fCG + J'*lamda);
%     else
%         dxdt(n/2+1:n) = M\(fCG + J'*lamda + tau);
%     end
end