%% State Variable declarations
syms x y phi alphaR betaR alphaL betaL vx vy vphi valphaR vbetaR valphaL vbetaL
q = [x; y; phi; alphaR; betaR; alphaL; betaL];
vq = [vx; vy; vphi; valphaR; vbetaR; valphaL; vbetaL];
%% Parameters
% System
syms m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3
% Environmental 
syms g
param = [m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g];
%% Calculations
% Mass & Joint position calculations
    % Body
CoGB = [x;y];
Hip = CoGB + [lH*sin(phi);
    - lH*cos(phi)];
    % Right Foot 
CoGTR = Hip + [l2*sin(phi+alphaR);
    -l2*cos(phi+alphaR)];
KneeR = Hip + [lL2*sin(phi+alphaR);
    -lL2*cos(phi+alphaR)];
CoGSR = KneeR + [l3*sin(phi+alphaR+betaR);
    - l3*cos(phi+alphaR+betaR)];
FootR = KneeR + [lL3*sin(phi+alphaR+betaR);
    - lL3*cos(phi+alphaR+betaR)];
    % Left Foot
CoGTL = Hip + [l2*sin(phi+alphaL);
    -l2*cos(phi+alphaL)];
KneeL = Hip + [lL2*sin(phi+alphaL);
    -lL2*cos(phi+alphaL)];
CoGSL = KneeL + [l3*sin(phi+alphaL+betaL);
    - l3*cos(phi+alphaL+betaL)];
FootL = KneeL + [lL3*sin(phi+alphaL+betaL);
    - lL3*cos(phi+alphaL+betaL)];

% Mass velocity calculations
vCoGB = jacobian(CoGB,q)*vq;
vCoGTR = jacobian(CoGTR,q)*vq;
vCoGSR = jacobian(CoGSR,q)*vq;
vCoGTL = jacobian(CoGTL,q)*vq;
vCoGSL = jacobian(CoGSL,q)*vq;

% Lagrangian Calculations
T = 0.5*( m1*sum(vCoGB.^2) + J1*vphi^2 ...
        + m2*sum(vCoGTR.^2) + J2*(vphi+valphaR)^2 ...
        + m3*sum(vCoGSR.^2) + J3*(vphi+valphaR+vbetaR)^2 ...
        + m2*sum(vCoGTL.^2) + J2*(vphi+valphaL)^2 ...
        + m3*sum(vCoGSL.^2) + J3*(vphi+valphaL+vbetaL)^2);
U = m1*CoGB(2)*g + m2*CoGTR(2)*g + m3*CoGSR(2)*g + m2*CoGTL(2)*g + m3*CoGSL(2)*g;
L = simplify(T-U);

dL_dq = jacobian(L,q).';
dL_dqdt = jacobian(L,vq).';
ddL_dqdt_dt = jacobian(dL_dqdt,q)*vq;

% Getting EOM terms
M = jacobian(dL_dqdt,vq).';
M = simplify(M);
fCG = simplify(dL_dq - ddL_dqdt_dt);

% The equations of motion are given with these functions as:   
% M * dqddt = f_cg(q, dqdt) + tau;
