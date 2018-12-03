function tau = groundController(x,phase)
n = size(x,1);
tau = zeros(n/2,1);

q = x(1:n/2,:);
dq = x(n/2+1:n,:);

n_u = 4;

%% parameters
param = yumingParameters();
sysParam = param.sysParam;
sysParam_minCoord = param.sysParam_minCoord;

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
    
% Note that you need to use the EoM with minimal coordinates

% Get qm and dqm
    % q_in order:  betaStance alphaStance phi alphaSwing betaSwing
if phase==1 || phase==2 || phase==3 || phase==7
    q_in = [q(5);q(4);q(3);q(6);q(7)];
    dq_in = [dq(5);dq(4);dq(3);dq(6);dq(7)];
else
    q_in = [q(7);q(6);q(3);q(4);q(5)];
    dq_in = [dq(7);dq(6);dq(3);dq(4);dq(5)];
end    
qm = q_MinCoord(q_in,pi);
dqm = dq_MinCoord(dq_in,pi);
xm = [qm;dqm];

% Get u_star
M = MassMatrix_MinCoord(qm,sysParam_minCoord);
fCG = FCorGrav_MinCoord(xm,sysParam_minCoord);
B=zeros(5,4);   B(2:3,1:2)=-1*eye(2);
                B(4:5,3:4)=   eye(2);
                % This definition of B gives the input in this order:
                % st_hip, st_knee, sw_hip, sw_knee

d_yDot_ddq = d_yDot_ddq_vector(xm,sysParam_minCoord);
d_yDot_dq_times_dqdt = d_yDot_dq_times_dqdt_vector(xm,sysParam_minCoord);

L_f_2_y = d_yDot_dq_times_dqdt + d_yDot_ddq * (M\fCG);
L_g_L_f_y =                      d_yDot_ddq * (M\B  );

u_star = - L_g_L_f_y\L_f_2_y;

%% Feedback control

y = y_output_vector(xm,sysParam_minCoord);
dy = dy_output_vector(xm,sysParam_minCoord);
eta = [y;dy];

% CLF_QP 
mu = CLF_QP(n_u, eta, L_g_L_f_y, u_star)

%% Assign output
u = u_star + L_g_L_f_y\mu;

% re-ordering 
if phase==1 || phase==2 || phase==3 || phase==7
    tau(4:7) = u;
else
    tau(4:7) = u([3 4 1 2]);
end    

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