%% This code generates the equations of motion for a two-knee hopper 
% during stance phase with minimal coordinates (not floating space)

%% Declare symbolic variables and calculate EoM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; 
restoredefaultpath

declare_syms_and_calc_EoM_minCoord;% A script file
                                   % Took this part out of this file, so that 
                                   % other files can also use it. 
%% Conversion from qm to q
q_float_partial = [betaStance; alphaStance; phi; alphaSwing; betaSwing];
dq_float_partial = [dbetaStance; dalphaStance; dphi; dalphaSwing; dbetaSwing];

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(M,'file','Functions\MassMatrix_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(fCG,'file','Functions\FCorGrav_MinCoord','vars',{[qm;dqm],sysParam_minCoord});

% Geometry 
matlabFunction(CoGStanceShank,'file','Functions\CoGStanceShank_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(StanceKnee,'file','Functions\StanceKnee_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(CoGStanceThigh,'file','Functions\CoGStanceThigh_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(Hip,'file','Functions\Hip_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(CoGB,'file','Functions\CoGB_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(CoGSwingThigh,'file','Functions\CoGSwingThigh_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(SwingKnee,'file','Functions\SwingKnee_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(CoGSwingShank,'file','Functions\CoGSwingShank_MinCoord','vars',{qm,sysParam_minCoord});
matlabFunction(SwingFoot,'file','Functions\SwingFoot_MinCoord','vars',{qm,sysParam_minCoord});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Represent the minimal coordinates in terms of floating base coord.
clear;

syms betaStance alphaStance phi alphaSwing betaSwing real
q_float_partial = [betaStance; alphaStance; phi; alphaSwing; betaSwing];
syms dbetaStance dalphaStance dphi dalphaSwing dbetaSwing real
dq_float_partial = [dbetaStance; dalphaStance; dphi; dalphaSwing; dbetaSwing];
 
syms PI

%% Calculation 
qm5 = betaSwing;
qm4 = alphaSwing;
qm3 = 2*PI - alphaStance;
qm2 = -betaStance;
qm1 = phi - qm2 - qm3 + 2*PI;

qm = [qm1; qm2; qm3; qm4; qm5];
dqm = jacobian(qm,q_float_partial)*dq_float_partial;

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(qm,'file','Functions\q_MinCoord','vars',{q_float_partial,PI});
matlabFunction(dqm,'file','Functions\dq_MinCoord','vars',{dq_float_partial,PI});


