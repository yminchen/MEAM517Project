function dxdt = dynamicsDoubleIntegrator(x,u,n_x)
    
    dxdt = zeros(n_x,1);
    
    dxdt(1) = x(2);
    dxdt(2) = u;

end