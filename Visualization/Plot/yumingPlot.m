
%% plot the output y
isPlotOutputY = true;

if isPlotOutputY
    sysParam_minCoord = param.sysParam_minCoord;
    y = [];
    for i = 1:numel(DS)
        phase = DS(i);
        q = S(i,1:7)';
        dq = S(i,8:14)';

        if phase==1 || phase==2 || phase==3 || phase==7
            isRightLegStance = true;
        else
            isRightLegStance = false;
        end

        if isRightLegStance
            theta = approx_leg_angle(q(3),q(4),q(5)); 
        else
            theta = approx_leg_angle(q(3),q(6),q(7)); 
        end
        if theta > param.theta_max || theta < param.theta_min 
            theta = max(param.theta_min,min(param.theta_max,theta));
        end    

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

        y_temp = y_output_theta_vector(xm,theta,sysParam_minCoord);
        y = [y;y_temp'];
    end
    figure
    min_height = min(min(y))*1.2;
    max_height = max(max(y))*1.2;
    axis([T(1) T(end) min_height max_height]);
    plotPhaseZone_singleStanceWalk;
    
    hold on;
    plot(T,y) 
    xlabel('Time (s)')
    ylabel('Output (rad)')
%     legend('Right Hip Error', 'Right Knee Error', 'Left Hip Error', 'Left Knee Error')
end

%% Plot leg angle
isPlotLegAngle = 0;
if 1 
    theta_min = param.theta_min;
    theta_max = param.theta_max;
    
    figure; 
    thetaR = approx_leg_angle(S(:,3)',S(:,4)',S(:,5)');
    min_height = min(thetaR);
    max_height = max(thetaR);
    axis([T(1) T(end) min_height max_height]);
    plotPhaseZone_singleStanceWalk;
    
    hold on;
    plot(T,thetaR)
    plot([T(1), T(end)],[theta_min, theta_min],'r')
    plot([T(1), T(end)],[theta_max, theta_max],'r')
    title('Right leg angle')
    xlabel('Time (s)')
    
    
    figure; 
    thetaL = approx_leg_angle(S(:,3)',S(:,6)',S(:,7)');
    
    min_height = min(thetaL);
    max_height = max(thetaL);
    axis([T(1) T(end) min_height max_height]);
    plotPhaseZone_singleStanceWalk;
    
    hold on;
    plot(T,thetaL)
    plot([T(1), T(end)],[theta_min, theta_min],'r')
    plot([T(1), T(end)],[theta_max, theta_max],'r')
    title('Left leg angle')

end










%%%%%%%%%%%%%%%%%%%%%%%%%% Old plotting function %%%%%%%%%%%%%%%%%%%%%%%%%%


%% initialize settings
%P = [S, L_R, dL_R, E, E_des, tau_R, F_c,  Theta_R, dTheta_R, FootPos_R, tau_L]

% generate flags indicating which plot should be shown.
plot_flag = zeros(1,n_plot);
if size(plot_flag_index,2) == 0
    plot_flag = zeros(1,n_plot);
else    
    for i = 1:n_plot
        for j = 1:size(plot_flag_index,2)
            if plot_flag_index(j) == i
                plot_flag(i) = 1;
            end
        end
    end
end

%% Generate the data we want to use
n = size(T,1);

% length of the virtual spring
dL_R = zeros(n,1);
L_R = zeros(n,1);
if plot_flag(15) || plot_flag(16)
    for i = 1:n
        % length of the virtual spring
        L_R(i) = SpringLengthR(S(i,1:7)',sysParam);
        dL_R(i) = dSpringLengthR(S(i,:)',sysParam);
    end
end

% total energy
E = zeros(n,1);
if plot_flag(17)
    for i = 1:n
        % length of the virtual spring
        E(i) = energy(S(i,:)',sysParam);
    end
end
% desired energy
%TODO: if it's uneven terrain, them this part should be modified.
E_des = zeros(n,1);
if plot_flag(18)
    m_tot = sysParam(1)+2*sysParam(4)+2*sysParam(8);
    E_des_temp = m_tot*9.81*(param.H+Terrain(S(1,1)+S(1,8)*param.t_prev_stance/2,param.ter_i))...
                + 0.5*m_tot*dx_des(1)^2;
    for i = 2:n
        if (DS(i)==1) && (DS(i-1)~=DS(i)) 
            E_des_temp = m_tot*9.81*(param.H+Terrain(S(i,1)+S(i,8)*t_prev_stance(i)/2,param.ter_i))...
                + 0.5*m_tot*dx_des(i)^2;
        end
        E_des(i) = E_des_temp;
    end
end

% right hip/knee torque
tau_R = zeros(n,2);
tau_L = zeros(n,2);
if plot_flag(19)||plot_flag(20)||plot_flag(27)||plot_flag(28)
    for i = 1:n
        if DS(i) == 1 || DS(i) == 4
            tau_temp = flightController(S(i,:)',DS(i));
            tau_R(i,:) = tau_temp(4:5,1)';
            tau_L(i,:) = tau_temp(6:7,1)';
        else
            tau_temp = groundController(S(i,:)',DS(i));
            tau_R(i,:) = tau_temp(4:5,1)';
            tau_L(i,:) = tau_temp(6:7,1)';
        end
    end
end

% contact force
F_c = zeros(n,2);   
if plot_flag(21)||plot_flag(22)
    for i = 1:n
        if DS(i)~=1 && DS(i)~=4
            tau_temp = groundController(S(i,:)',DS(i));

            M = MassMatrix(S(i,1:7)',sysParam);
            if DS(i)==2 || DS(i)==3
                J = JcontPointR(S(i,1:7)',sysParam);
                dJ = dJcontPointR(S(i,:)',sysParam);
            elseif DS(i)==5 || DS(i)==6
                J = JcontPointL(S(i,1:7)',sysParam);
                dJ = dJcontPointL(S(i,:)',sysParam);
            end
            fCG = FCorGrav(S(i,:)',sysParam);
            lamda = -pinv(J*(M\J'))*(J*(M\(fCG+tau_temp)) + dJ*S(i,8:14)');
            F_c(i,:) = lamda';
        end
    end
end

% theta and d_theta
theta = zeros(n,1);
d_theta = zeros(n,1);   
if plot_flag(23)||plot_flag(24)
    for i = 1:n
        theta(i) = ThetaR_BF(S(i,1:7)',sysParam);
        d_theta(i) = dThetaR_BF(S(i,:)',sysParam);
    end
end

% Foot position (used to check the impact mapping)
FootPos_R = zeros(n,2);
if plot_flag(25)
    for i = 1:n
        FootPos_R(i,:) = posFootR(S(i,1:7)',sysParam)';
    end
end


% put all the data into P
P = [S L_R dL_R E E_des tau_R F_c theta d_theta FootPos_R tau_L];

% height of the phase zone
max_height = 0;
min_height = 0;
for i = 1:size(plot_flag,2)
    if plot_flag(i)
        for j = 1:n
            if P(j,i)>max_height
                max_height = P(j,i);
            elseif P(j,i)<min_height
                min_height = P(j,i);
            end
        end
    end
end
max_height = max_height+0.1*(max_height-min_height);
min_height = min_height-0.1*(max_height-min_height);

%% first plot
figure;
axis([T(1) T(end) min_height max_height]);

% plot states
% plotStates;    
% legend('show');

legend('show');
% plot phase
plotPhaseZone_singleStanceWalk;

% plot states 
plotStates;    

% others
hold on
plot([T(1) T(end)], [0 0], 'k--' ,'LineWidth',1)
hold off

% figure name
title(plotTitle)
xlabel('Time (s)')

%% second plot (trajectory)
Total_CoG = zeros(n,2);
for i = 1:n
    Total_CoG(i,:) = CoG_tot(S(i,1:7)',sysParam)';
end
    
figure;

plot(Total_CoG(:,1),Total_CoG(:,2),'b'); % CoG of total mass
% plot(P(:,1),P(:,2),'b'); % CoG of main body

axis equal;
title('Trayjectory of Two-leg Hopper');
xlabel(' (m)');
ylabel(' (m)');
