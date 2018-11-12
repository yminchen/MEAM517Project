%% This code generates the function for the desired hip angle during flight phase
clear; clc; 

% The derivation is in the scrap paper of one leg hopper.

%% Defination
syms theta phi beta 
input = [theta phi beta]; 
% Parameters
syms lH lL2 lL3
param = [lH lL2 lL3];

%% Calculations

A = lH*sin(phi+theta);

C = A*lL2+lL3*lH*sin(phi+theta)*cos(beta);
D = lL3*lH*sin(phi+theta)*sin(beta);
p = asin(D/(C^2+D^2)^0.5);
phi_prime = p + asin(A*lH*sin(phi+theta)/(C^2+D^2)^0.5);

alpha = phi + theta + phi_prime;
alpha = simplify(alpha);

%% Create functions
if ~exist('Functions','dir')
    mkdir('Functions');
end

matlabFunction(alpha,'file','Functions\alpha_desired','vars',[input,param]);
