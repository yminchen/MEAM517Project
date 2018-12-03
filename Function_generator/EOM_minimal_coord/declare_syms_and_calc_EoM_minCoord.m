%% State Variable declarations

% Mininal coordinates
syms qm1 qm2 qm3 qm4 qm5 dqm1 dqm2 dqm3 dqm4 dqm5 real
qm = [qm1; qm2; qm3; qm4; qm5];
dqm = [dqm1; dqm2; dqm3; dqm4; dqm5];

%% Parameters
% System
syms m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 real
% Environmental 
syms g real
% Math
syms PI real
sysParam_minCoord = [m1 J1 lH m2 J2 l2 lL2 m3 J3 l3 lL3 g PI];

%% Represent some of the floating base coordinates
betaStance = -qm2; 
alphaStance = 2*PI - qm3;
phi = qm1 + qm2 + qm3 - 2*PI;
alphaSwing = qm4; 
betaSwing = qm5;

dbetaStance = jacobian(betaStance,qm)*dqm;
dalphaStance = jacobian(alphaStance,qm)*dqm;
dphi = jacobian(phi,qm)*dqm;
dalphaSwing = jacobian(alphaSwing,qm)*dqm;
dbetaSwing = jacobian(betaSwing,qm)*dqm;

%% Calculations
% Assume stance leg is at origin

% Mass & Joint position calculations

% Until Stance knee
CoGStanceShank = [-(lL3-l3)*sin(qm1);
                   (lL3-l3)*cos(qm1)];
StanceKnee = [-lL3*sin(qm1);
               lL3*cos(qm1)];
% Until Hip
CoGStanceThigh = StanceKnee + [-(lL2-l2)*sin(qm1+qm2);
                                (lL2-l2)*cos(qm1+qm2)];
Hip = StanceKnee + [-lL2*sin(qm1+qm2);
                     lL2*cos(qm1+qm2)];
% Until Body
CoGB = Hip + [-lH*sin(phi);
               lH*cos(phi)];
% Until Swing knee
CoGSwingThigh = Hip + [l2*sin(phi+alphaSwing);
                       -l2*cos(phi+alphaSwing)];
SwingKnee = Hip + [lL2*sin(phi+alphaSwing);
                  -lL2*cos(phi+alphaSwing)];
% Until Swing foot
CoGSwingShank = SwingKnee + [l3*sin(phi+alphaSwing+betaSwing);
                            -l3*cos(phi+alphaSwing+betaSwing)];
SwingFoot = SwingKnee + [lL3*sin(phi+alphaSwing+betaSwing);
                        -lL3*cos(phi+alphaSwing+betaSwing)];

% Mass velocity calculations
dCoGStanceShank = jacobian(CoGStanceShank,qm)*dqm;
dCoGStanceThigh = jacobian(CoGStanceThigh,qm)*dqm;
dCoGB = jacobian(CoGB,qm)*dqm;
dCoGSwingThigh = jacobian(CoGSwingThigh,qm)*dqm;
dCoGSwingShank = jacobian(CoGSwingShank,qm)*dqm;

% Lagrangian Calculations
T = 0.5*( m3*sum(dCoGStanceShank.^2) + J3*(dphi+dalphaStance+dbetaStance)^2 ...
        + m2*sum(dCoGStanceThigh.^2) + J2*(dphi+dalphaStance)^2 ...
        + m1*sum(dCoGB.^2)           + J1*dphi^2 ...
        + m2*sum(dCoGSwingThigh.^2)  + J2*(dphi+dalphaSwing)^2 ...
        + m3*sum(dCoGSwingShank.^2)  + J3*(dphi+dalphaSwing+dbetaSwing)^2);
U =   m3*CoGStanceShank(2)*g ...
    + m2*CoGStanceThigh(2)*g ...
    + m1*CoGB(2)*g ...
    + m2*CoGSwingThigh(2)*g ...
    + m3*CoGSwingShank(2)*g;
L = simplify(T-U);

dL_dq = jacobian(L,qm).';
dL_dqdt = jacobian(L,dqm).';
ddL_dqdt_dt = jacobian(dL_dqdt,qm)*dqm;

% Getting EOM terms
M = jacobian(dL_dqdt,dqm).';
M = simplify(M);
fCG = simplify(dL_dq - ddL_dqdt_dt);

% The equations of motion are given with these functions as:   
% M * dqddt = f_cg(q, dqdt) + tau;
