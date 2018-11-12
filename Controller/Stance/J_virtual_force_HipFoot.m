function tau = J_virtual_force_HipFoot(theta,f)
% This is derived in Pratt's paper. 
params = simParameters();
L1 = params(11);
L2 = params(7);
theta_a = theta(1);
theta_k = theta(2);

J11 = (-L1*L2*sin(theta_a))/(L1*cos(theta_a)+L2*cos(theta_a+theta_k));
J12 = (-L1*cos(theta_a))/(L1*cos(theta_a)+L2*cos(theta_a+theta_k));
tau = [J11 J12;0 -1]*f;

end