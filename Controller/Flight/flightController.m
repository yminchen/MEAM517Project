function tau = flightController(x,phase)
% Front foot is tuned
n = size(x,1);
tau = zeros(n/2,1);

%% parameters 
param = yumingParameters();
sysParam = param.sysParam;
lH = param.lH;
lL2 = param.lL2;
lL3 = param.lL3;



%% Limit
tau_max = param.tau_max;
if param.torque_limit_flag
    for i = 4:7
        if abs(tau(i))>tau_max
            tau(i) = sign(tau(i))*tau_max;
        end
    end
end


end