function dxdt = doubleStanceDyn(t,x,phase,t_prev_stance,k_des,dx_des)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    dxdt = zeros(size(x));
    n = size(x,1);
    dxdt(1:n/2) = x(n/2+1:n);
    
    % controller
%     tau = groundController(x,phase,t_prev_stance,k_des,dx_des);
    tau = zeros(n/2,1);
    
    % Calculate Ground Reaction Force
    M = MassMatrix(x(1:n/2),sysParam);
    fCG = FCorGrav(x,sysParam);

    J = [JcontPointR(x(1:n/2),sysParam);
         JcontPointL(x(1:n/2),sysParam)];
    dJ = [dJcontPointR(x,sysParam);
          dJcontPointL(x,sysParam)];
    
    lamda = -(J*(M\J'))\(J*(M\(fCG+tau)) + dJ*x(n/2+1:n));

    dxdt(n/2+1:n) = M\(fCG + J'*lamda + tau);
    
end