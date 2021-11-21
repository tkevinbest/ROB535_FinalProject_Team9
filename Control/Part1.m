clear
clc

%% Vehicle Parameters
% Input limits 
delta_lim = [-0.5, 0.5];
Fx_lim = [-5000, 5000];

% Constants
m = 1400;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy = 1.2;
Dy = 0.7;
Ey = -1.6;
Shy = 0;
Svy = 0;
g = 9.806;

% Initial conditions
Z_init = [287; 5; -176; 0; 2; 0];

% Load in data about track (boundaries, centerline, and centerline orientation)
load('TestTrack.mat');
