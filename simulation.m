% TODO: 

% You can tune the CLF-QP controller by looking at the output

% Check if your theta saturation is working in the function. (feed into the function with 
% theta and also without theta)

% Now the transition from stance to flight is disabled. Turn this back on in the future

% You can get a better trajectory. Currently the swing knee kind of over
% extend right before touchdown.

% You can test whether the torque saturation is working or not by setting
% torque limit in the QP to be a lower value.

% You can:
% plot the knee joint velocity out 
% plot the input out
% plot the contact force out

% Change quadprog to SNOPT. See if the speed can be improved significantly.

% Tune the paramter c3 in the CLF_QP function

% For robust CLF-QP, you can calculate whether the delta_H and delta_G exceed the bound.
% Test if the controller fail when delta_G and delta_H is out of the predifined maximum bound. 

%% Some minor housekeeping 
clear; clc; clf; close all;
restoredefaultpath;     % In case, the old version of quadprog is called...
addpath('Functions','Dynamics','Events',...
    'Controller','Controller/Flight','Controller/Stance','Controller/CLF-QP',...
    'Visualization','Visualization/Animation/','Visualization/Animation/Terrain',...
    'Visualization/Plot');
%% Flags 
F_PLOT = 1;             % Plot system response
F_SAVEPLOT = 0;         % Save generated plot
F_ANIMATE = 1;          % Animate system response
F_SAVEVID = 0;          % Save generated animation   
%% Simulation control
relTol  = 1e-6;%1e-10;         % Relative tolerance: Relative tolerance for ode45 numerical integration
absTol  = 1e-6;%1e-10;         % Absolute tolerance: Absolute tolerance for ode45 numerical integration 
dt      = 0.01; %[s]    % Max time step: Maximum time step for numerica integration 
tFinal  = 3;    %[s]    % Simulation end time

%% Yu-ming's parameters
param = yumingParameters();
dx_des = 0;         % desired speed (initialized to be 0)

%% Initial condition
sim_init_condition;

x0 = [x0;y0;phi0;alphaR0;betaR0;alphaL0;betaL0];
dx0 = [vx0;vy0;vphi0;valphaR0;vbetaR0;valphaL0;vbetaL0];

sysParam = param.sysParam;

rightFootPotition = posFootR(x0,sysParam);
leftFootPotition = posFootL(x0,sysParam);
disp(['Right foot height = ',num2str(rightFootPotition(2)),' (m)'])
disp(['left foot height = ',num2str(leftFootPotition(2)),' (m)'])
if rightFootPotition(2) < 0
    disp('ERROR: Right foot is below the ground in the begining')
end
if leftFootPotition(2) < 0
    disp('ERROR: Left foot is below the ground in the begining')
end

%% Simulation (the simulation is not prefect at multiple contacts)
tic

T(1) = 0;
S(1,:) = [x0;dx0];

DS(1) = 2;  % right leg: flight compression thrust 1 2 3
            % left  leg: flight compression thrust 4 5 6
            % right leg double stance: 7
            % left leg double stance: 8
while T(end) < tFinal
    tspan = T(end):dt:tFinal;
    
    % Flight phase
    if(DS(end) == 1) || (DS(end) == 4)
        fltSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,...
            'Events',@(t,x) flightEvent(t,x,DS(end)),'MaxStep',dt);    
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) flightDyn(t,x,DS(end)),...
            [tspan, tspan(end)+dt],S(end,:),fltSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        disp(['Current simulation time: ',num2str(Tp(end))]);
        
        if(isempty(Ie)== 1) % Simulation timed out
            disp('Time out');
        elseif(size(Ie,2)==2)
            if(Ie(1) == 1 && Ie(2) == 2)
                disp('Both Feet Touchdown at the same time');
                [dqplus,~] = doubleStanceImpactUpdate(Sp(end,:)',DS(end));
                Sp(end,:) = [Sp(end,1:7)'; dqplus];
                DS(end) = 7;
            end
        elseif(Ie(1) == 1) % Touchdown event
            disp('Right Foot Touchdown');
            DS(end) = 2;
            
            dqplus = singleStanceImpactUpdate(Sp(end,:)',DS(end));
            Sp(end,:) = [Sp(end,1:7)'; dqplus];
        elseif(Ie(1) == 2) % Touchdown event
            disp('left Foot Touchdown');
            DS(end) = 5;
            
            dqplus = singleStanceImpactUpdate(Sp(end,:)',DS(end));
            Sp(end,:) = [Sp(end,1:7)'; dqplus];
        elseif(Ie(1) == 3 || Ie(1) == 4 || Ie(1) == 5 || Ie(1) == 6)
            disp('Contact point is not front foot');
            % Save the state before terminting the simulation
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        else 
            disp('Flight Phase: Invalid event code');
        end
        
    % Single stance phase 
    elseif (DS(end)== 2) || (DS(end)== 3) || (DS(end)== 5) || (DS(end)== 6)
        singleStSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,...
            'Events',@(t,x) singleStanceEvent(t,x,DS(end),dx_des),'MaxStep',dt);
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) singleStanceDyn(t,x,DS(end),...
            dx_des),[tspan, tspan(end)+dt],S(end,:),singleStSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        disp(['Current simulation time: ',num2str(Tp(end))]);
        
        if(isempty(Ie) == 1) % Simulation timed out
            disp('Time out');
        elseif(Ie(end) == 1) % Takeoff event
            if (DS(end)==5)|| (DS(end)==6)
                disp('Left Foot Takeoff. Enter Flight');
                DS(end) = 1;
            elseif (DS(end)==2)|| (DS(end)==3)
                disp('Right Foot Takeoff. Enter Flight');
                DS(end) = 4;
            end
        elseif(Ie(end) == 2)
            [dqplus,nextPhase] = doubleStanceImpactUpdate(Sp(end,:)',DS(end));
            Sp(end,:) = [Sp(end,1:7)'; dqplus];
            DS(end) = nextPhase;
        elseif(Ie(end) == 3 || Ie(end) == 4 || Ie(end) == 5 || Ie(end) == 6)
            disp('Contact point is not stance foot');
            % Save the state before terminting the simulation
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        elseif Ie(end) == 7
            disp('ERROR: Contact force was negative');
            % Save the state before terminting the simulation
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        else 
            disp('Ground Phase: Invalid event code');
        end
        
    % Double stance phase
    elseif (DS(end)== 7) || (DS(end)== 8) 
        doubleStSimOpts = odeset('RelTol',relTol,'AbsTol',absTol,...
            'Events',@(t,x) doubleStanceEvent(t,x,DS(end),dx_des),'MaxStep',dt);
        [Tp,Sp,TEp,SEp,Ie] = ode45(@(t,x) doubleStanceDyn(t,x,DS(end),...
            dx_des),[tspan, tspan(end)+dt],S(end,:),doubleStSimOpts);
        sz = size(Sp,1);
        DS = [DS;DS(end)*ones(sz-1,1)];
        disp(['Current simulation time: ',num2str(Tp(end))]);
        
        if(isempty(Ie) == 1) % Simulation timed out
            disp('Time out');
        elseif(Ie(end) == 1) % Liftoff event
            disp('Right Foot Liftoff. Enter Left Single Stance.');
            DS(end) = 5;
        elseif(Ie(end) == 2) % Liftoff event
            disp('Left Foot Liftoff. Enter Right Single Stance.');
            DS(end) = 2;
        elseif(Ie(end) == 3 || Ie(end) == 4 || Ie(end) == 5 || Ie(end) == 6)
            disp('Contact point is not stance foot');
            % Save the state before terminting the simulation
%             S = [S;Sp(2:sz,:)];
%             T = [T;Tp(2:sz,:)];
%             break;
        elseif Ie(end) == 7
            disp('ERROR: Contact force was negative');
            % Save the state before terminting the simulation
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
        
    % Simulation bug testing
    RightFoot = posFootR(Sp(end,:)',sysParam); 
    LeftFoot = posFootL(Sp(end,:)',sysParam); 
    if (RightFoot(2)<-0.01) || (LeftFoot(2)<-0.01)
        disp('Foot penatrated the ground. Simulation bug somewhere');
        disp(DS)
        break
    end
    
end

time_to_run_simulation = toc

%% Plot 
plotTitle = 'CLF-QP with Torque Saturation';

% Ref: P = [S, L_R, dL_R, E, E_des, tau_R, F_c,  Theta_R, dTheta_R, FootPos_R, tau_L]
%           14 15   16    17 18     19/20  21/22 23       24        25/26      27/28
n_plot = 28;
plot_flag_index = [3 10 19 20 21 22];
plot_flag_index = [3 10 19 20 ];   % look at phi and joint torque
plot_flag_index = [3 10 ];     % look at phi
plot_flag_index = [19 20 ];   % look at right leg torque
% plot_flag_index = [27 28 ];   % look at left leg torque
% plot_flag_index = [17 18];    % look at energy
% plot_flag_index = [19 20 23 24]; % tune PD controller for theta in flight
% plot_flag_index = [4 11 19];    % tune PD controller for hip in flight
% plot_flag_index = [4  ];    % tune PD controller for hip in flight
% plot_flag_index = [5  12 19 20];    % tune PD controller for knee in flight
% plot_flag_index = [5  12];
% plot_flag_index = [15 16];    % look at spring length and speed
% plot_flag_index = [8];        % x velocity
% plot_flag_index = [1 8];      % x
% plot_flag_index = [21 22];    % gound reaction force
% plot_flag_index = [25 26];    % Foot position
% plot_flag_index = [2];        % y

if F_PLOT
    yumingPlot;
end


    
%% Animation
videoTitle = plotTitle;
videoFileName = [videoTitle,'.avi'];
if F_ANIMATE
    Animation(T,S,T(end),F_SAVEVID,videoFileName,videoTitle);    
end
