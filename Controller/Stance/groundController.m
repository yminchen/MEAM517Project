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

epsilon = 0.01;     % control the converge rate
                    % converges fast when epsilon is small
                    
% (Kp, Kd, epsilon) = (10, 2, 0.01) is good. (better than 10, 2, 0.1)

%% Decide which leg is the stance leg
if phase==1 || phase==2 || phase==3 || phase==7
    isRightLegStance = true;
else
    isRightLegStance = false;
end

%% Testing (see if the leg angle exceeds the bound)
if isRightLegStance
    theta = approx_leg_angle(x(3),x(4),x(5)); 
else
    theta = approx_leg_angle(x(3),x(6),x(7)); 
end
if theta > param.theta_max || theta < param.theta_min 
    theta = theta
end    

%% Nominal control (input output feedback linearization)
    % L_f_2_h   = d_yDot_dq_times_dqdt + d_yDot_ddq * (M^-1) * fCG
    % L_g_L_f_h =                        d_yDot_ddq * (M^-1) * B 
    
% Note that you need to use the EoM with minimal coordinates

% Get qm and dqm
    % q_float_partial order:  betaStance alphaStance phi alphaSwing betaSwing
if isRightLegStance
    q_float_partial = [q(5);q(4);q(3);q(6);q(7)];
    dq_float_partial = [dq(5);dq(4);dq(3);dq(6);dq(7)];
else
    q_float_partial = [q(7);q(6);q(3);q(4);q(5)];
    dq_float_partial = [dq(7);dq(6);dq(3);dq(4);dq(5)];
end    
qm = q_MinCoord(q_float_partial,pi);
dqm = dq_MinCoord(dq_float_partial,pi);
xm = [qm;dqm];

% Get u_star
M = MassMatrix_MinCoord(qm,sysParam_minCoord);
fCG = FCorGrav_MinCoord(xm,sysParam_minCoord);
B=zeros(5,4);   B(2,2)=-1;
                B(3,1)=-1;
                B(4:5,3:4)=eye(2);
                % This definition of B gives the input in this order:
                % st_hip, st_knee, sw_hip, sw_knee

d_yDot_ddq = d_yDot_ddq_vector(xm,sysParam_minCoord);
d_yDot_dq_times_dqdt = d_yDot_dq_times_dqdt_vector(xm,sysParam_minCoord);

L_f_2_y = d_yDot_dq_times_dqdt + d_yDot_ddq * (M\fCG);
L_g_L_f_y =                      d_yDot_ddq * (M\B  );

u_star = - L_g_L_f_y\L_f_2_y;

%% Feedback control on output
control_option = 2; % 0: no feedback
                    % 1: PD feedback control
                    % 2: CLF_QP

y = y_output_vector(xm,sysParam_minCoord);
dy = dy_output_vector(xm,sysParam_minCoord);
eta = [y;dy];
if control_option == 0      % no feedback control
    mu = zeros(n_u,1);
elseif control_option == 1  % PD feedback control
    mu = [-Kp/epsilon^2, -Kd/epsilon]*eta;
elseif control_option == 2  % CLF_QP 
    mu = CLF_QP(n_u, epsilon, eta, L_g_L_f_y, u_star);
else
    disp('ERROR in controller selection');
end

%% Assign output
u = u_star + L_g_L_f_y\mu;

% re-ordering 
if isRightLegStance
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