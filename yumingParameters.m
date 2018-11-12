function param = yumingParameters()

%% Environment parameters
param.ter_i = 0;         % terrain label (0 is flag terrain)

%% Joint mass, inertia and length parameters
% Main body segment
param.m1 = 7.9026;       %[kg]         Mass of main body segment 
param.J1 = 0.08;         %[kgm^2]      Rotational inertia of main body segment 
param.lH = 0.138;        %[m]          Distance from main body CoM to hip center

% Thigh segments
param.m2 = 0.7887;       %[kg]         Mass of thigh segment
param.J2 = 0.002207;     %[kgm^2]      Rotational inertia of thigh segment
param.l2 = 0.0193399;    %[m]          Distance from hip center to thigh CoM
param.lL2 = 0.2;         %[m]          Distance from hip center to knee center

% Shank segments
param.m3 = 0.510;        %[kg]         Mass of shank segment
param.J3 = 0.006518;     %[kgm^2]      Rotational inertia of thigh segment
param.l3 = 0.165235;     %[m]          Distance from knee center to shank CoM
param.lL3 = 0.2385;      %[m]          Distance from knee center to foot center

% Environmental constants
param.g = 9.81;          %[m/s^2]      Accelerations due to gravity
sysParam = [param.m1 param.J1 param.lH ...
            param.m2 param.J2 param.l2 param.lL2 ...
            param.m3 param.J3 param.l3 param.lL3 param.g];
param.sysParam = sysParam;

%% Joint and actuator constraint parameters
% Hip constraint
param.alpha_min = -pi/2;  %[rad]        Minimum hip angle
param.alpha_max = pi/2;   %[rad]        Maximum hip angle

% Knee constraint
param.beta_min = -3*pi/4; %[rad]        Minimum knee angle
param.beta_max = 0;       %[rad]        Maximum knee angle

% Motor constraints
param.T_rot_max = 0.568;  %[Nm]         Maximum rotor torque
param.du_rot_max = 838;   %[rad/s]      Maximum rotor velocity torque

% Joint torque constraint
param.tau_max = 25;       %[Nm]         Maximum rotor torque
param.torque_limit_flag = 1;


%% Virtual Component (a spring between hip and foot)
param.k = 5000;%5000;            % spring constant   
param.d = 10;             % spring damping 
param.beta_eq = -20*pi/180;
                          % knee relax angle
                          
% When the spring is between body and foot: (L_sp0 need to be adjusted)
param.L_sp0 = sysParam(3) + (sysParam(7)^2+sysParam(11)^2 ...
               -2*sysParam(7)*sysParam(11)*cos(pi+param.beta_eq))^0.5;  
% When the spring is between CoG and foot: (L_sp0 need to be adjusted)
param.L_sp0_GF = 0.5288;  % spring original length (m) 
                          % (when theta=0 and knee bends beta_eq)    
% When the spring is between hip and foot: (L_sp0 need to be adjusted)
param.L_sp0_HF = (sysParam(7)^2+sysParam(11)^2 ...
               -2*sysParam(7)*sysParam(11)*cos(pi+param.beta_eq))^0.5; 
param.theta2 = asin(sysParam(11)*sin(-param.beta_eq)/param.L_sp0_HF);

%% controller parameters
param.target_pos = 6;
param.t_prev_stance = 0.2/(param.k/5000); 
param.H = 0.6;%0.63, 0.6;            % desired height (effecting kp_rai and kp_pos!)
               % We need to set it to a value a little bit higher becuase
               % we didn't take the rotational energy into account when
               % calculating the desired energy.
param.max_dx_des = 1.2;     % maximum of desired speed (not real speed)
        % 0.9 for running backward
        % seems like 1.2/1.3 is the limit (with H=0.6m)
        % max_dx_des can go higher, but then it also jumps higher.
% param.dx_des = 0;         % desired speed (initialized to be 0)
% param.E_low = 0;          % energy at lowest point (initialized to be 0)
% param.E_des = 0;          % desired energy (initialized to be 0)
% param.L_sp_low = 0;       % spring length when mass reaches lowest height 
% param.k_des = 0;          % desired spring constant during the thrust phase
% param.x_td = 0;           % state vector at previous touchdown 
% param.prev_t = 0;

% position controller parameters
kp_pos = 2;       
        % kp_pos depends on max_dx_des.
kd_pos = 2; 
% Raibert controller parameter
kp_rai = 0.04;      % Raibert sytle controller
        % kp_rai depends on H (the height) and k (the stiffness).
        % For larger desired speed, you need higher kp_rai.
        % But the larger the kp_rai is, the more unstable when changing speed.
        % When it's stable enough, you can try to increase max_dx_des.
param.k_f = [kp_pos kd_pos kp_rai];  % f stands for flight


%% finite state machine
param.lLeg_flag = 0;      % 0 is right leg, and 1 is left leg.
param.phase = 0;          % 0 is flight phase, and 1 is stance phase
param.postApex_flag = 0;  % Flight phase: 0 is before Apex, and 1 is after Apex.
param.thrust_flag = 0;    % 0 is in thrust phase, 1 is not.
              
%% plotting parameters
param.line_height = 7;

end
