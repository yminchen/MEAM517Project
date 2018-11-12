%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('Functions','Dynamics','Events',...
    'Controller','Controller/Flight','Controller/Stance',...
    'Visualization','Visualization/Animation/','Visualization/Animation/Terrain',...
    'Visualization/Plot');
%% Flags 
F_PLOT = 1;             % Plot system response
F_SAVEPLOT = 0;         % Save generated plot
F_ANIMATE = 1;          % Animate system response
F_SAVEVID = 0;          % Save generated animation   
%% Simulation control
relTol  = 1e-6;         % Relative tolerance: Relative tolerance for ode45 numerical integration
absTol  = 1e-6;         % Absolute tolerance: Absolute tolerance for ode45 numerical integration 
dt      = 0.01; %[s]    % Max time step: Maximum time step for numerica integration 
tFinal  = 5;    %[s]    % Simulation end time

%% Simulation parameters
x0 = 0;          %[m]    % initial X position 
y0 = 0.6;        %[m]    % initial Y position
body_rot = 0;
phi0 = body_rot;          %[rad]  % initial angle between vertical and hip
alphaR0 = -0.5+body_rot;     %[rad]  % iniial angle between hip and thigh
betaR0 = 0+body_rot;      %[rad]  % initial angle between thigh and shank
alphaL0 = 0.5+body_rot;     %[rad]  % iniial angle between hip and thigh
betaL0 = 0+body_rot;      %[rad]  % initial angle between thigh and shank
vx0 = 0;         %[m/s]  % initial X velociy 
vy0 = 0;         %[m/s]  % initial Y velociy
vphi0 = 0;       %[rad/s]% initial phi angular velocity
valphaR0 = 0;    %[rad/s]% initial alpha angular  velocity
vbetaR0 = 0;     %[rad/s]% initial beta angular  velocity
valphaL0 = 0;    %[rad/s]% initial alpha angular  velocity
vbetaL0 = 0;     %[rad/s]% initial beta angular  velocity

%% Yu-ming's parameters
param = yumingParameters();
t_prev_stance = param.t_prev_stance;   %[s]
t_prev_stance_forPlot = [t_prev_stance 0]; 
prev_t = 0;
dx_des = 0;         % desired speed (initialized to be 0)
dx_des_forPlot = [dx_des 0];
E_low = 0;          % energy at lowest point (initialized to be 0)
E_des = 0;          % desired energy (initialized to be 0)
L_sp_low = 0;       % spring length when mass reaches lowest height 
k_des = 0;          % desired spring constant during the thrust phase
k_des_forPlot = [k_des 0];
x_td = 0;           % state vector at previous touchdown 
% Ground contact Flag
contactR = 0;       % true if right foot is touching the ground
contactL = 0;       % true if left foot is touching the ground

%% Simulation 
%Setting up simulation
sysParam = param.sysParam;

T(1) = 0;
S(1,:) = [x0;y0;phi0;alphaR0;betaR0;alphaL0;betaL0;...
          vx0;vy0;vphi0;valphaR0;vbetaR0;valphaL0;vbetaL0];

DS(1) = 1;  % right leg: flight compression thrust 1 2 3
            % left  leg: flight compression thrust 4 5 6
while T(end) < tFinal
    tspan = T(end):dt:tFinal;
    if(DS(end) == 1) || (DS(end) == 4)
        fltSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,...
            'Events',@(t,x) flightEvent(t,x,DS(end)),'MaxStep',dt);    
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) flightDyn(t,x,t_prev_stance,DS(end)),...
            [tspan, tspan(end)+dt],S(end,:),fltSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        
%         disp(size(Ie));
        if(isempty(Ie)== 1) % Simulation timed out
            disp('Time out');
        elseif(Ie(1) == 1) % Touchdown event
            if DS(end) == 1
                disp('Right Foot Touchdown');
                DS(end) = 2;
            elseif DS(end) == 4
                disp('left Foot Touchdown');
                DS(end) = 5;
            end
            qplus = singleStanceImpactUpdate(Sp(end,:)',DS(end));
            Sp(end,:) = [Sp(end,1:7)'; qplus];
            % Yu-Ming's parameter
            prev_t = Tp(end);
            % flight controller info
            dx_des = -param.k_f(1)*((Sp(end,1)+Sp(end,8)*t_prev_stance/2)-param.target_pos)...
                     -param.k_f(2)*Sp(end,8);
            if dx_des>param.max_dx_des
                dx_des = param.max_dx_des;
            elseif dx_des<-param.max_dx_des
                dx_des = -param.max_dx_des;
            end
            dx_des_forPlot = [dx_des_forPlot;dx_des Tp(end)]; 
            % state vector at touch down
            x_td = Sp(end,:);
            % length speed at touch down
            if DS(end) == 2
                dL = abs(dSpringLengthR(Sp(end,:)',sysParam)); 
            elseif DS(end) == 5
                dL = abs(dSpringLengthL(Sp(end,:)',sysParam)); 
            end
        elseif(Ie(1) == 2 || Ie(1) == 3 || Ie(1) == 4 || Ie(1) == 5 || Ie(1) == 6)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            disp('Contact point is not front foot');
            break;
        else 
            disp('Flight Phase: Invalid event code');
        end
    elseif (DS(end)~=1) && (DS(end)~=4)
        gndSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,...
            'Events',@(t,x) singleStanceEvent(t,x,DS(end),t_prev_stance,k_des,dx_des),'MaxStep',dt);
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) singleStanceDyn(t,x,DS(end),...
            t_prev_stance,k_des,dx_des),[tspan, tspan(end)+dt],S(end,:),gndSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        
        %disp(size(Ie));
        if(isempty(Ie) == 1) % Simulation timed out
            disp('Time out');
        elseif(Ie(end) == 1) % Takeoff event
            if DS(end)==6
                disp('Left Foot Takeoff');
                DS(end) = 1;
            elseif DS(end)==3
                disp('Right Foot Takeoff');
                DS(end) = 4;
            end
            % Yu-Ming's parameter
            t_prev_stance = Tp(end) - prev_t;
            t_prev_stance_forPlot = [t_prev_stance_forPlot; t_prev_stance Tp(end)]; 
        elseif(Ie(end) == 2) % Reach the lowest height
            if DS(end)==2
                disp('Right Foot Thrust');
                DS(end) = 3;
            elseif DS(end)==5
                disp('Left Foot Thrust');
                DS(end) = 6;
            end
            % calculate spring length and energy at lowest height
            if DS(end)==3
                L_sp_low = SpringLengthR(Sp(end,1:7)',sysParam);
            elseif DS(end)==6
                L_sp_low = SpringLengthL(Sp(end,1:7)',sysParam);
            end
            E_low = energy(Sp(end,:)',sysParam)+0.5*param.k*(L_sp_low - param.L_sp0)^2;
            % calculate desired energy (not including swing leg's)
            m_tot = sysParam(1)+2*sysParam(4)+2*sysParam(8);
            E_des = m_tot*9.81*(param.H+Terrain(Sp(end,1)+Sp(end,8)*t_prev_stance/2,param.ter_i))...
                    + 0.5*m_tot*dx_des^2;
                    %+ pi/4*param.d*dL*(param.L_sp0-L_sp_low);
            
            k_des = param.k + 2*(E_des-E_low)/(param.L_sp0-L_sp_low)^2;
%             k_des = 2*(E_des-E_low)/(param.L_sp0-L_sp_low)^2; % Testing
%             if k_des < param.k
%                 k_des = param.k;
%                 %disp('here');
%             end
            k_des_forPlot = [k_des_forPlot; k_des Tp(end)];
        elseif(Ie(end) == 3 || Ie(end) == 4 || Ie(end) == 5 || Ie(end) == 6 || Ie(end) == 7)
            S = [S;Sp(2:sz,:)];
            T = [T;Tp(2:sz,:)];
            disp('Contact point is not stance foot');
            break;
        elseif Ie(end) == 8
            disp('ERROR: Contact force was negative');
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        else 
            disp('Ground Phase: Invalid event code');
        end
    else
        disp('State machine not working.')
    end
    S = [S;Sp(2:sz,:)];
    T = [T;Tp(2:sz,:)];
end

%% Plot 

% Ref: P = [S, L_R, dL_R, E, E_des, tau_R, F_c,  Theta_R, dTheta_R, FootPos_R, tau_L]
%           14 15   16    17 18     19/20  21/22 23       24        25/26      27/28
n_plot = 28;
plot_flag_index = [3 10 19 20 21 22];
plot_flag_index = [3 10 19 20 ];   % look at phi and joint torque
plot_flag_index = [3 10 ];     % look at phi
plot_flag_index = [19 20 ];   % look at joint torque
plot_flag_index = [17 18];    % look at energy
% plot_flag_index = [19 20 23 24]; % tune PD controller for theta in flight
% plot_flag_index = [4 11 19];    % tune PD controller for hip in flight
% plot_flag_index = [4  ];    % tune PD controller for hip in flight
% plot_flag_index = [5  12 19 20];    % tune PD controller for knee in flight
% plot_flag_index = [5  12];
% plot_flag_index = [15 16];    % look at spring length and speed
plot_flag_index = [8];        % x velocity
% plot_flag_index = [1 8];      % x
plot_flag_index = [21 22];    % gound reaction force
% plot_flag_index = [25 26];    % Foot position
% plot_flag_index = [2];        % y

if F_PLOT
    yumingPlot;
end

%% Animation
videoFileName = 'Two_Leg_Hopper_with_Knee.avi';
if F_ANIMATE
    Animation(T,S,DS,T(end),F_SAVEVID,videoFileName);    
end
