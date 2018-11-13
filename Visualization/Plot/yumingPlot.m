
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

% get dx_des
dx_des = zeros(n,1);
for i = 1:n
    for j = 2:size(dx_des_forPlot,1)+1
        if j==size(dx_des_forPlot,1)+1
            dx_des(i) = dx_des_forPlot(j-1,1);
            break;
        end
        if T(i)<dx_des_forPlot(j,2)
            dx_des(i) = dx_des_forPlot(j-1,1);
            break;
        end
    end
end

% get k_des
k_des = zeros(n,1);
for i = 1:n
    for j = 2:size(k_des_forPlot,1)+1
        if j == size(k_des_forPlot,1)+1
            k_des(i) = k_des_forPlot(j-1,1);
            break;
        end
        if T(i)<k_des_forPlot(j,2)
            k_des(i) = k_des_forPlot(j-1,1);
            break;
        end
    end
end

%% Generate the data we want to see

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
            tau_temp = flightController(S(i,:)',t_prev_stance(i),DS(i));
            tau_R(i,:) = tau_temp(4:5,1)';
            tau_L(i,:) = tau_temp(6:7,1)';
        else
            tau_temp = groundController(S(i,:)',DS(i),k_des(i),dx_des(i));
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
            tau_temp = groundController(S(i,:)',DS(i),k_des(i),dx_des(i));

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
plotStates;    
legend('show');

% plot states 
plotStates;    

% others
hold on
plot([T(1) T(end)], [0 0], 'k--' ,'LineWidth',1)
hold off

% figure name
title('Two-leg Hopper')
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
