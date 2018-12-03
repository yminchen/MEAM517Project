%% This code generates the equations of motion for a two-knee hopper 
% during stance phase with minimal coordinates (not floating space)

%% Declare symbolic variables and calculate EoM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; 
restoredefaultpath

declare_syms_and_calc_EoM_minCoord;% A script file
                                   % Took this part out of this file, so that 
                                   % other files can also use it. 

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(M,'file','Functions\MassMatrix_MinCoord','vars',{qm,param});
matlabFunction(fCG,'file','Functions\FCorGrav_MinCoord','vars',{[qm;dqm],param});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Represent the minimal coordinates in terms of floating base coord.
clear;

syms betaStance alphaStance phi alphaSwing betaSwing real
q = [betaStance; alphaStance; phi; alphaSwing; betaSwing];
syms dbetaStance dalphaStance dphi dalphaSwing dbetaSwing real
dq = [dbetaStance; dalphaStance; dphi; dalphaSwing; dbetaSwing];
 
syms PI
param = PI;

%% Calculation 
qm5 = betaSwing;
qm4 = alphaSwing;
qm3 = 2*PI - alphaStance;
qm2 = -betaStance;
qm1 = phi - qm2 - qm3 + 2*PI;

qm = [qm1; qm2; qm3; qm4; qm5];
dqm = jacobian(qm,q)*dq;

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(qm,'file','Functions\q_MinCoord','vars',{q,param});
matlabFunction(dqm,'file','Functions\dq_MinCoord','vars',{dq,param});


