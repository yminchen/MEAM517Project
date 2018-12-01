function tau = groundController(x,phase)
n = size(x,1);
tau = zeros(n/2,1);

q = x(1:n/2,:);
dq = x(n/2+1:n,:);

%% parameters
param = yumingParameters();
sysParam = param.sysParam;

% pd gains
Kp = param.Kp;
Kd = param.Kd;

%% Nominal control (input output feedback linearization)
    % L_f_2_h   = d_yDot_dq_times_dqdt + d_yDot_ddq * (M^-1) * fCG
    % L_g_L_f_h =                        d_yDot_ddq * (M^-1) * B 

M = MassMatrix(q,sysParam);
fCG = FCorGrav(x,sysParam);
B=zeros(7,4); B(4:7,:)=eye(4);

d_yDot_ddq = d_yDot_ddq_vector(x,sysParam);
d_yDot_dq_times_dqdt = d_yDot_dq_times_dqdt_vector(x,sysParam);

L_f_2_h = d_yDot_dq_times_dqdt + d_yDot_ddq * (M\fCG);
L_g_L_f_h =                      d_yDot_ddq * (M\B  );

u_star = - L_g_L_f_h\L_f_2_h

%% Assign output
tau(4:7) = u_star;

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