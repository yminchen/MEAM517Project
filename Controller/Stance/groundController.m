function tau = groundController(x,phase)
n = size(x,1);
tau = zeros(n/2,1);

%% parameters
param = yumingParameters();
sysParam = param.sysParam;

% pd gains
Kp = param.Kp;
Kd = param.Kd;




%% Limit
tau_max = param.tau_max;
if param.torque_limit_flag
    for i = 4:7
        if abs(tau(i))>tau_max
            tau(i) = sign(tau(i))*tau_max;
        end
    end
end

%% Testing

end