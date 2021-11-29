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

% Reference trajectories
U_ref = 0;
Y_ref = 0;

%% Symbolics to find A and B
% syms delta real
% syms x u y v psi r real
% syms i real
% syms Fx Fyf Fyr real
% 
% dx = u*cos(psi) - v*sin(psi);
% du = (1/m) * (-f*m*g + Nw*Fx - Fyf*sin(delta)) + v*r;
% dy = u*sin(psi) + v*cos(psi);
% dv = (1/m) * (Fyf*cos(delta) + Fyr) - u*r;
% dpsi = r;
% dr = (1/Iz)*(a*Fyf*cos(delta) - b*Fyr);
% 
% dz = [dx; du; dy; dv; dpsi; dr];
% z = [x; u; y; v; psi; r];
% input = [delta; Fx];
% 
% A_sym = jacobian(dz,z)
% B_sym = jacobian(dz,input)

psi = Y_ref(5,i)
u = Y_ref(2,i)
v = Y_ref(4,i)
r = Y_ref(6,i)

A_fun = @(i) [0, cos(Y_ref(5,i)), 0, -sin(Y_ref(5,i)), - Y_ref(4,i)*cos(Y_ref(5,i)) - Y_ref(2,i)*sin(Y_ref(5,i)),           0
              0,               0, 0,       Y_ref(6,i),                                                         0,  Y_ref(4,i)
              0, sin(Y_ref(5,i)), 0,  cos(Y_ref(5,i)),   Y_ref(2,i)*cos(Y_ref(5,i)) - Y_ref(4,i)*sin(Y_ref(5,i)),           0
              0,     -Y_ref(6,i), 0,                0,                                                         0, -Y_ref(2,i)
              0,               0, 0,                0,                                                         0,           1
              0,               0, 0,                0,                                                         0,           0];

B_fun = @(i) [                        0,     0
                 -(Fyf*cos(delta))/1400, 1/700
                                      0,     0
                 -(Fyf*sin(delta))/1400,     0
                                      0,     0
              -(9*Fyf*sin(delta))/17780,     0];

%% Number of decision variables for colocation method
npred=10;
Ndec = 3*(npred+1) + 2*(npred);