clear 

%% System
n_u = 4;
y = 0.1*ones(n_u,1);    % example output
dy = 0.1*ones(n_u,1);   % example output
eta = [y;dy];

Lg_Lf_y = eye(4);       % example 
u_star = [10;3;-2;4];   % example

%% CLF_QP(eta) 
mu = CLF_QP(n_u, eta, Lg_Lf_y, u_star);

