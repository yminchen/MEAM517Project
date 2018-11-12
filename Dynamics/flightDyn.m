function dxdt = flightDyn(t,x,t_prev_stance,phase)
    param = yumingParameters();
    sysParam = param.sysParam;
    
    n = size(x,1);
    dxdt = zeros(size(x));
    dxdt(1:n/2) = x(n/2+1:n);
    
    %controller
    tau = flightController(x,t_prev_stance,phase);
    
    dxdt(n/2+1:n) = MassMatrix(x(1:n/2),sysParam)\(FCorGrav(x,sysParam)+tau);
    
end