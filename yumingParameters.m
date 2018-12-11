function param = yumingParameters()

%% Environment parameters
param.ter_i = 0;         % terrain label (0 is flag terrain)

%% Joint mass, inertia and length parameters
% Main body segment
param.m1 = 7.9026;       %[kg]         Mass of main body segment 
param.J1 = 0.08;         %[kgm^2]      Rotational inertia of main body segment 
param.lH = 0.137675;     %[m]          Distance from main body CoM to hip center

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

sysParam_minCoord = [param.m1 param.J1 param.lH ...
            param.m2 param.J2 param.l2 param.lL2 ...
            param.m3 param.J3 param.l3 param.lL3 param.g pi];
param.sysParam_minCoord = sysParam_minCoord;

%% Actuator constraint parameters
n_u = 4;
param.n_u = n_u;

% Joint torque constraint
param.tau_max = 10;%25;       %[Nm]         Maximum rotor torque
param.torque_limit_flag = 1;            % 1 = enable motor torque limit

%% controller parameters
param.control_option = 1; % 1 is traditional HZD (use feedback linearization plus pd control)
                          % 2 is CLF QP
                          % 3 is robust CLF QP

% pd gains for traditional HZD controller
param.Kp = 10*eye(n_u);
param.Kd = 5*eye(n_u);
% A = [zeros(n_u) eye(n_u);-param.Kp, -param.Kd]; % This matrix has to be Hurwitz
% disp(eig(A)); 

% Min and max leg angle from nominal trajectory
% Don't load things here, cause it got load whenever you call yumingParamters (slows simulation down by a lot)
% load('Trajectory_optimization\Nomial_trajectories\Traj2\Data1Step_30-Nov-2018_100nodes.mat');
param.theta_min = -0.2867;
param.theta_max = 0.2652;

%% Others
param.target_pos = 6;


end
