function [mu] = CLF_QP(eta, Lg_Lf_y, u_star, torque_max, isTorqueSaturation, isRobustToModelPerturbation)
    n_u = size(eta,1)/2;
    
    u_min = -torque_max;
    u_max = torque_max;
    
    %% parameters for robust CLF_QP
    delta_H_max = 0.1; % seems like the bigger this is, the more unstable the controller is
    delta_G_max = 0.1;
    % TODO: you can calculate whether the delta_H and delta_G exceed the
    % bound.
    
    %% Choose the CLF    
    % Paramters
    epsilon = 0.01; % seems that smaller epsilon make the controller stable?
                    % at least 0.01 is better than 0.1 
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
    
    %% Derivatives of CLF
    L_f_V_epsilon = eta'*(F'*P_epsilon + P_epsilon*F)*eta;
    L_g_V_epsilon = 2*eta'*P_epsilon*G;
    
    phi_0_eplison = L_f_V_epsilon + c3/epsilon*V_epsilon;
    phi_1_eplison = L_g_V_epsilon;
    
    if isRobustToModelPerturbation
        tilde_phi_p_0_eplison = phi_0_eplison ...
                + 2*eta'*P_epsilon*[zeros(n_u,1);ones(n_u,1)]*delta_H_max;
        tilde_phi_n_0_eplison = phi_0_eplison ...
                - 2*eta'*P_epsilon*[zeros(n_u,1);ones(n_u,1)]*delta_H_max;
        tilde_phi_max_0_eplison = max(tilde_phi_p_0_eplison, ...
                                      tilde_phi_n_0_eplison);
                                  
        tilde_phi_p_1_eplison = phi_1_eplison ...
                + L_g_V_epsilon*delta_G_max;
        tilde_phi_n_1_eplison = phi_1_eplison ...
                - L_g_V_epsilon*delta_G_max;
    end
    
    %% Torque saturation constraints for the QP
    % Plug in values
    if isTorqueSaturation
        A_mu = Lg_Lf_y\eye(n_u);
        bumin = (u_min-u_star);
        bumax = (u_max-u_star);
    else
        A_mu = zeros(n_u);
        bumin = zeros(n_u,1);
        bumax = zeros(n_u,1);
    end
        
    %% Solve quadratic programs
    if ~isRobustToModelPerturbation
    %% CLF-QP
        % Parameter
        p = 1;%10000 % penalty on constraint (dV <= -k*V) violation 

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
    else
    %% Robust CLF-QP 
        % Parameter
        p_1 = 1;%10000 % penalty on constraint (dV <= -k*V) violation 
        p_2 = 1;
        
        % Formulate
        H = [eye(n_u)     zeros(n_u,2)  ; 
             zeros(2,n_u) [p_1 0;0 p_2] ];
        f = zeros(n_u+2,1);
        A = [tilde_phi_p_1_eplison  -1    0     ;
             tilde_phi_n_1_eplison   0   -1     ;
             -A_mu                  zeros(n_u,2);
              A_mu                  zeros(n_u,2)];
        b = [-tilde_phi_max_0_eplison*ones(2,1) ; 
             -bumin                             ; 
              bumax                             ];

        % Solve
    %     x = quadprog(H,f,A,b);
        Aeq=[];beq=[];lb=[];ub=[];x0=[];
        options = optimoptions('quadprog','Display','off'); % off, final
        x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);

        % extract solution
        mu = x(1:4);
        d_1 = x(5);   % constraint violation
        d_2 = x(6);   % constraint violation
        
    end
end

