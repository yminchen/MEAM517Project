
%% Fall from the air
% x0 = 0;          %[m]    % initial X position 
% y0 = 0.6;        %[m]    % initial Y position
% body_rot = 0;
% phi0 = body_rot;          %[rad]  % initial angle between vertical and hip
% alphaR0 = -0.5+body_rot;     %[rad]  % iniial angle between hip and thigh
% betaR0 = 0+body_rot;      %[rad]  % initial angle between thigh and shank
% alphaL0 = 0.6+body_rot;     %[rad]  % iniial angle between hip and thigh
% betaL0 = 0+body_rot;      %[rad]  % initial angle between thigh and shank
% vx0 = 0;         %[m/s]  % initial X velociy 
% vy0 = 0;         %[m/s]  % initial Y velociy
% vphi0 = 0;       %[rad/s]% initial phi angular velocity
% valphaR0 = 0;    %[rad/s]% initial alpha angular  velocity
% vbetaR0 = 0;     %[rad/s]% initial beta angular  velocity
% valphaL0 = 0;    %[rad/s]% initial alpha angular  velocity
% vbetaL0 = 0;     %[rad/s]% initial beta angular  velocity

%% Initial condition from nominal trajectory
load('Trajectory_optimization\Nomial_trajectories\Traj2\Data1Step_30-Nov-2018_100nodes.mat');

x0 = x1(1,1);           %[m]    % initial X position 
y0 = x1(1,2);           %[m]    % initial Y position
phi0 = x1(1,3);         %[rad]  % initial angle between vertical and hip
alphaR0 = x1(1,4);      %[rad]  % iniial angle between hip and thigh
betaR0 = x1(1,5);       %[rad]  % initial angle between thigh and shank
alphaL0 = x1(1,6);      %[rad]  % iniial angle between hip and thigh
betaL0 = x1(1,7);       %[rad]  % initial angle between thigh and shank
vx0 = xdot1(1,1);         %[m/s]  % initial X velociy 
vy0 = xdot1(1,2);         %[m/s]  % initial Y velociy
vphi0 = xdot1(1,3);       %[rad/s]% initial phi angular velocity
valphaR0 = xdot1(1,4);    %[rad/s]% initial alpha angular  velocity
vbetaR0 = xdot1(1,5);     %[rad/s]% initial beta angular  velocity
valphaL0 = xdot1(1,6);    %[rad/s]% initial alpha angular  velocity
vbetaL0 = xdot1(1,7);     %[rad/s]% initial beta angular  velocity


