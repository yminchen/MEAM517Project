function tau = flightController(x,t_prev_stance,phase)
% Front foot is tuned
n = size(x,1);
tau = zeros(n/2,1);

%% parameters 
param = yumingParameters();
sysParam = param.sysParam;
lH = param.lH;
lL2 = param.lL2;
lL3 = param.lL3;

%% Get the desired touch-down angle
% Position controller parameters
kp_pos = param.k_f(1);
kd_pos = param.k_f(2);
% Position controller (PD control)
dx_des = -kp_pos*(x(1)-param.target_pos) - kd_pos*x(8); % target_pos is fixed.
if dx_des>param.max_dx_des
    dx_des = param.max_dx_des;
elseif dx_des<-param.max_dx_des
    dx_des = -param.max_dx_des;
end

% dx_des = 1; % for tuning kp_rai
% dx_des = 0.1; % for tuning kp_rai
% dx_des = 0; % for testing

% Raibert style controller parameters
kp_rai = param.k_f(3);
max_theta_tar = 50*pi/180;
% Raibert style controller
x_des = x(1+n/2)*t_prev_stance/2 + kp_rai*(x(1+n/2)-dx_des);
theta_tar = asin(x_des/param.L_sp0);
if theta_tar > max_theta_tar
    theta_tar = max_theta_tar;
elseif theta_tar < -max_theta_tar
    theta_tar = -max_theta_tar;
end

% theta_tar = 1; % for tuning the next PD controller
% theta_tar = 0; % for debugging

%% Front Foot: Hip joint (Use "theta and desired theta" instead of alpha and desired alpha)
% % Derive current theta (spring is between Body and foot!)
% if phase == 1 
%     theta = ThetaR_BF(x(1:n/2),sysParam);
%     d_theta = dThetaR_BF(x,sysParam);
% elseif phase == 4
%     theta = ThetaL_BF(x(1:n/2),sysParam);
%     d_theta = dThetaL_BF(x,sysParam);
% end
% % PD controller parameters
% kp = 50;   %10;   %50;    % 120 % 300
% kd = 3.5;  %0.7;  %3.5;    % 4.8 % 8
% max_f = 1000;    % maximum torque that can be applied
% % PD controller for desired phi.
% err = theta - theta_tar;
% derr =  d_theta;     
%         %%% TODO: theta_target is dynamic, so I should modify derr.
% tau_hip = -kp*err - kd*derr;
% if tau_hip > max_f
%     tau_hip = max_f;
% elseif tau_hip < -max_f
%     tau_hip = -max_f;
% end
% 
% % Assignment
% if phase == 1
%     tau(4) = tau_hip;
% elseif phase == 4
%     tau(6) = tau_hip;
% end

%% Front Foot: Hip joint (Use alpha and desired alpha)
% I want to use the hip angle derived from touch-down angle, and see how it performs.
% (because in Simulink, torque control seems not possible. We can't apply negative torque)

% Derive target alpha 
if phase == 1
    beta = -x(5);
    beta = -param.beta_eq; % testing
elseif phase == 4
    beta = -x(7);
    beta = -param.beta_eq; % testing
end
l = (lL2^2 + lL3^2 - 2*lL2*lL3*cos(pi-beta))^0.5;
if x(3)==theta_tar
    alpha_tar = asin(lL3/l*sin(pi-beta));
else
    if x(3)<theta_tar
        delta = theta_tar-x(3);
        phi1 = asin(lH/l*sin(delta));
        phi2 = pi-delta-phi1;
        phi3 = asin(lL3/l*sin(pi-beta));
        alpha_tar = -phi2+phi3+pi;
    elseif  x(3)>theta_tar
        delta = x(3)-theta_tar;
        phi1 = asin(lH/l*sin(delta));
        phi2 = pi-delta-phi1;
        phi3 = asin(lL3/l*sin(pi-beta));
        alpha_tar = +phi2+phi3-pi;
    end
end

% for tunning
% alpha_tar = 1;

% PD controller parameters
% (Tuned)
kp = 130;   %50;   %10;   %50;    % 120 % 300
kd = 5;  %3.5;  %0.7;  %3.5;   % 4.8 % 8
% PD controller for desired phi.
if phase == 1
    err = x(4) - alpha_tar;
    derr =  x(11);     
elseif phase == 4
    err = x(6) - alpha_tar;
    derr =  x(13);     
end
        
tau_hip = -kp*err - kd*derr;

% Assignment
if phase == 1
    tau(4) = tau_hip;
elseif phase == 4
    tau(6) = tau_hip;
end

%% Front Foot: Knee joint
% PD controller parameters
% (Tuned)
kp = 100;  %50;   % 100  
kd = 1;  %0.65;  % 0.9
% PD controller for desired phi.
if phase == 1
    err = x(5) - param.beta_eq;
    derr =  x(5+n/2);      
elseif phase == 4
    err = x(7) - param.beta_eq;
    derr =  x(7+n/2);   
end
tau_knee = -kp*err - kd*derr;

% Assignment
if phase == 1
    tau(5) = tau_knee;
elseif phase == 4
    tau(7) = tau_knee;
end

%% Rear Foot: Hip joint
%%%%%%%% Way I: foot pointing to the ground %%%%%%%%%%%%%
% Derive current theta (angle between virtical line and "Body-foot"!)
% if phase == 1 
%     theta = ThetaL_BF(x(1:n/2),sysParam);
%     d_theta = dThetaL_BF(x,sysParam);
% elseif phase == 4
%     theta = ThetaR_BF(x(1:n/2),sysParam);
%     d_theta = dThetaR_BF(x,sysParam);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Way II: knee pointing to the ground %%%%%%%%%%%%
% Derive current theta (angle between virtical line and "Hip-Knee"!)
if phase == 1 
    theta = ThetaL_HK(x(1:n/2),sysParam);
    d_theta = dThetaL_HK(x,sysParam);
elseif phase == 4
    theta = ThetaR_HK(x(1:n/2),sysParam);
    d_theta = dThetaR_HK(x,sysParam);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PD controller parameters
kp = 10;   %50; % 120 % 300
kd = 0.7;  %3.5; % 4.8 % 8
% PD controller for desired theta.
% 1.
theta_tar = pi/9;       % TODO: This could be the angle at previous liftoff
    % pi/9 all the time
% 2. 
% theta_tar = -1/18*pi*2*dx_des + pi/6;
    % pi/9 if dx_des = 0.5 
    % pi/6 if dx_des = 0

err = theta - theta_tar;
derr =  d_theta;     
        %%% TODO: If theta_target is dynamic, I should modify derr.

tau_hip = -kp*err - kd*derr;


% % Knee pointing to ground can actully use the following code.============
% (We don't need theta, just use hip angle directly.)
% alpha_tar = -x(3) + pi/9;
% d_alpha_tar = -x(10);%0;%-x(10);
% if phase == 1
%     err = x(6) - alpha_tar;
%     derr =  x(13) - d_alpha_tar;   
% elseif phase == 4
%     err = x(4) - alpha_tar;
%     derr =  x(11) - d_alpha_tar;   
% end
% kp = 10;%120;
% kd = 0.7;%5;
% tau_hip = -kp*err - kd*derr;
% % =======================================================================

% Assignment
if phase == 1
    tau(6) = tau_hip;
elseif phase == 4
    tau(4) = tau_hip;
end


%% Rear Foot: Knee joint
% PD controller parameters
kp = 50;    
kd = 0.65;   
% PD controller for desired phi.
if phase == 1
    err = x(7) - param.beta_eq*3;
    derr =  x(7+n/2);      
elseif phase == 4
    err = x(5) - param.beta_eq*3;
    derr =  x(5+n/2);   
end
tau_knee = -kp*err - kd*derr;

% Assignment
if phase == 1
    tau(7) = tau_knee;
elseif phase == 4
    tau(5) = tau_knee;
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

%% testing
% tau(4) = 0;
% tau(6) = 0;
% tau(7) = 0;

% tau(1)=0;
% tau(2)=0;
% tau(3)=0;
% % tau(4)=0;
% tau(6)=0;
% tau(7)=0;

end