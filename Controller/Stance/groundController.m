function tau = groundController(x,phase)
n = size(x,1);
tau = zeros(n/2,1);

q = x(1:n/2,:);
dq = x(n/2+1:n,:);

n_u = 4;

%% parameters
param = yumingParameters();
sysParam = param.sysParam;

% pd gains
Kp = param.Kp;
Kd = param.Kd;



%% Testing

% Testing (it's assume that the stance foot is the rigt foot)
theta = approx_leg_angle(x(3),x(4),x(5)); 
if theta > param.theta_max || theta < param.theta_min 
    theta = theta
end    



%% Nominal control (input output feedback linearization)
    % L_f_2_h   = d_yDot_dq_times_dqdt + d_yDot_ddq * (M^-1) * fCG
    % L_g_L_f_h =                        d_yDot_ddq * (M^-1) * B 

M = MassMatrix(q,sysParam);
fCG = FCorGrav(x,sysParam);
B=zeros(7,4); B(4:7,:)=eye(4);

d_yDot_ddq = d_yDot_ddq_vector(x,sysParam);
d_yDot_dq_times_dqdt = d_yDot_dq_times_dqdt_vector(x,sysParam);

L_f_2_y = d_yDot_dq_times_dqdt + d_yDot_ddq * (M\fCG);
L_g_L_f_y =                      d_yDot_ddq * (M\B  );

u_star = - L_g_L_f_y\L_f_2_y

%% Feedback control

% y = 0.1*ones(n_u,1);    % example output
% dy = 0.1*ones(n_u,1);   % example output
% eta = [y;dy];
% 
% % CLF_QP 
% mu = CLF_QP(n_u, eta, L_g_L_f_y, u_star)

%% Assign output
tau(4:7) = u_star;
% tau(4:7) = u_star + mu;

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