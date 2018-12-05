%% Some minor housekeeping 
clear; clc; clf; close all;
addpath('../Functions',...
    '../Visualization/Animation/','../Visualization/Animation/Terrain',...
    '../');

%% Flags 
F_ANIMATE = 1;          % Animate system response
F_SAVEVID = 0;          % Save generated animation   

%% Load data
% load('Nomial_trajectories/Traj1/Data1Step_29-Nov-2018.mat');
% load('Nomial_trajectories/Traj2/Data1Step_30-Nov-2018_100nodes.mat');
load('Nomial_trajectories/Traj3/Data1Step_04-Dec-2018.mat');

n = size(x1,1);
T = [0:t1/(n-1):t1]';
S = [x1, xdot1];

%% Animation
videoTitle = 'Nominal Trajecotry';
videoFileName = [videoTitle,'.avi'];
if F_ANIMATE
    Animation(T,S,T(end),F_SAVEVID,videoFileName,videoTitle);    
end
