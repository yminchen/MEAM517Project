function [F] = userFuncDoubleIntegrator(z,n_sample,n_x,n_u,T)
    nCon   = n_x*(n_sample-1);
    nF     = nCon + 1;
    n_X = n_x * n_sample;
    
    dt = T/(n_sample-1);
    
    %% F
    F = zeros(nF,1);
    
    % Dynamics Constraints
    for i = 1:n_sample-1
        x_i =      z([1:n_x]+n_x*(i-1),1);
        x_iplus1 = z([1:n_x]+n_x*i    ,1);
        u_i =  z(n_X+[1:n_u]+n_u*(i-1),1);
        
        dxdt = dynamicsDoubleIntegrator(x_i,u_i,n_x);
        
        F([1:n_x]+n_x*(i-1),1) = x_iplus1 - x_i - dxdt*dt;
    end
    
    % Cost
    F(nF) = z(n_X-1)^2 + z(n_X-2)^2;
    
    %% G
    
end