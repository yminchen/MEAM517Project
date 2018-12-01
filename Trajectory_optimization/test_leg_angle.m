clear
load('Nomial_trajectories/Traj2/Data1Step_30-Nov-2018_100nodes.mat')

addpath('../Functions');
addpath('../');

%%
p = yumingParameters;
p_vec = p.sysParam;

%% Calculate the leg angle
theta = [];
for i = 1:100
temp = ThetaR_HF(x1(i,:).', p_vec);
theta = [theta; temp];
end

%% Plot
t = 0:t1/(size(x1,1)-1):t1;
t = t';

figure; plot(t, theta)

%% Testing
% posFtR = [];
% posHp = [];
% for i = 1:100
% FootTemp = posFootR(x1(i,:).', p_vec);
% posFtR = [posFtR; FootTemp.'];
% HipTemp = posHip(x1(i,:).', p_vec);
% posHp = [posHp; HipTemp.'];
% end
% 
% %atan2((FootR(1)-Hip(1)),(Hip(2)-FootR(2)))
% atan2(posFtR(1)-posHp(1),posHp(2)-posFtR(2))

%% Rough leg angle
theta_approx = [];
for i = 1:100
temp = -(x1(i,3) + x1(i,4) + x1(i,5)*0.5);
theta_approx = [theta_approx; temp];
end

hold on; plot(t, theta_approx)
legend('Leg angle (ground truth)', 'Approximated leg angle')

