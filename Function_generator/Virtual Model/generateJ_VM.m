%% This code generates the transformation matrix of virtual force
clear; clc; 

%% Defination
syms alpha beta phi_f
theta = [phi_f beta alpha]; 
% Parameters
syms lH lL2 lL3
param = [lH lL2 lL3];

%% Calculations
% Body pose
x = lL3*sin(phi_f)+lL2*sin(phi_f+beta)+lH*sin(phi_f+beta+alpha);
y = lL3*cos(phi_f)+lL2*cos(phi_f+beta)+lH*cos(phi_f+beta+alpha);
phi = -1*(phi_f+beta+alpha);
X = [x;y;phi];

J = jacobian(X,theta);

J_reduced = [J(2,2)-J(1,2)*J(2,1)/J(1,1), J(3,2)-J(1,2)*J(3,1)/J(1,1);
             J(2,3)-J(1,3)*J(2,1)/J(1,1), J(3,3)-J(1,3)*J(3,1)/J(1,1)];
J_reduced = simplify(J_reduced);

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(J_reduced,'file','Functions\J_virtual_force','vars',[theta,param]);
