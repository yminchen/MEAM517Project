function [mu] = CLF_QP(n_u, eta, Lg_Lf_y, u_star)
    u_min = -25;
    u_max = 25;
    
    %% Choose the CLF    
    % Paramters
    epsilon = 0.01;
    c3 = 0.001; % Need to tune this! 
                % (I think for small enough number, there exsits a solution for the QP)
    
    Kp = 10*eye(n_u);   % P gain for PD control (used to chose CLF)
    Kd = 2*eye(n_u);    % D gain for PD control (used to chose CLF)
    Q = eye(2*n_u);

    % Solve for Lyapunov equation
    A = [zeros(n_u) eye(n_u); -Kp -Kd];
    P = lyap(A',Q);
    
    % IO linearized dynamics
    F = [zeros(n_u), eye(n_u); zeros(n_u), zeros(n_u)];
    G = [zeros(n_u); eye(n_u)];
    
    % CLF
    P_epsilon = [eye(4)/epsilon zeros(4); zeros(4) eye(4)]*P*...
                [eye(4)/epsilon zeros(4); zeros(4) eye(4)];
    V_epsilon = eta'*P_epsilon*eta;
    
    % Derivatives of CLF
    L_f_V_epsilon = eta'*(F'*P_epsilon + P_epsilon*F)*eta;
    L_g_V_epsilon = 2*eta'*P_epsilon*G;
    
    phi_0_eplison = L_f_V_epsilon + c3/epsilon*V_epsilon;
    phi_1_eplison = L_g_V_epsilon;

    %% CLF-QP
    % Parameter
    p = 1;%10000 % penalty on constraint (dV <= -k*V) violation 
    isTorqueSaturation = false;
    
    % Plug in values
    A_mu = eye(n_u);
    bumin = Lg_Lf_y*(u_min-u_star);
    bumax = Lg_Lf_y*(u_max-u_star);
    if ~isTorqueSaturation
        A_mu = zeros(n_u);
        bumin = zeros(n_u,1);
        bumax = zeros(n_u,1);
    end
    
    % Formulate
    H = [eye(n_u)     zeros(n_u,1); 
         zeros(1,n_u) p            ];
    f = zeros(n_u+1,1);
    A = [phi_1_eplison  -1;
         -A_mu          zeros(n_u,1);
          A_mu          zeros(n_u,1)];
    b = [-phi_0_eplison; 
         -bumin        ; 
          bumax         ];

    % Solve
%     x = quadprog(H,f,A,b);
    Aeq=[];beq=[];lb=[];ub=[];x0=[];
    options = optimoptions('quadprog','Display','off'); % off, final
    x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
    
    % extract solution
    mu = x(1:4);
    d = x(5);   % constraint violation
    
end

