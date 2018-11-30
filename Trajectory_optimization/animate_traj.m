%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('../Functions','../Dynamics','../Events',...
    '../Controller','../Controller/Flight','../Controller/Stance',...
    '../Visualization','../Visualization/Animation/','../Visualization/Animation/Terrain',...
    '../Visualization/Plot',...
    '../');

%% Flags 
F_ANIMATE = 1;          % Animate system response
F_SAVEVID = 0;          % Save generated animation   

%% Load data
load('Nomial_trajectories/Traj1/Data1Step_29-Nov-2018.mat');

n = size(x1,1);
T = [0:t1/(n-1):t1]';
S = [x1, xdot1];

%% Animation
videoFileName = 'CLF-QP.avi';
if F_ANIMATE
    Animation(T,S,T(end),F_SAVEVID,videoFileName);    
end