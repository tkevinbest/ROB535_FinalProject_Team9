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

% psi = Y_ref(5,i)
% u = Y_ref(2,i)
% v = Y_ref(4,i)
% r = Y_ref(6,i)
% delta = U_Ref(1,i)


% Pre-calculate
delta = U_ref(1,:);
u = Y_ref(2,:); v = Y_ref(4,:); r = Y_ref(4,:);
% Slip angles
alpha_f = delta - atan(v + a*r ./ u);
alpha_r = - atan(v - b*r ./ u);
% 
phi_yf = (1 - Ey) * (alpha_f + Shy) + (Ey/By)*atan(By*(alpha_f + Shy));
phi_yr = (1 - Ey) * (alpha_r + Shy) + (Ey/By)*atan(By*(alpha_r + Shy));
% Forces
Fzf = (b / (a + b)) * m * g;
Fyf = Fzf * Dy * sin(Cy*atan(By*phi_yf)) + Svy;
Fzr = (a / (a + b)) * m * g;
Fyr = Fzr * Dy * sin(Cy*atan(By*phi_yr)) + Svy;


% Discrete time A and B matrices
A = @(i) eye(6) +  dt*[0, cos(Y_ref(5,i)), 0, -sin(Y_ref(5,i)), - Y_ref(4,i)*cos(Y_ref(5,i)) - Y_ref(2,i)*sin(Y_ref(5,i)),           0
                       0,               0, 0,       Y_ref(6,i),                                                         0,  Y_ref(4,i)
                       0, sin(Y_ref(5,i)), 0,  cos(Y_ref(5,i)),   Y_ref(2,i)*cos(Y_ref(5,i)) - Y_ref(4,i)*sin(Y_ref(5,i)),           0
                       0,     -Y_ref(6,i), 0,                0,                                                         0, -Y_ref(2,i)
                       0,               0, 0,                0,                                                         0,           1
                       0,               0, 0,                0,                                                         0,           0];

B = @(i) dt*[                                 0,     0
                 -(Fyf(i)*cos(U_Ref(1,i)))/1400, 1/700
                                              0,     0
                 -(Fyf(i)*sin(U_Ref(1,i)))/1400,     0
                                              0,     0
              -(9*Fyf(i)*sin(U_Ref(1,i)))/17780,     0];

%% Number of decision variables for colocation method
npred=10;
Ndec = 3*(npred+1) + 2*(npred);

%% Testing Equality Constraints
idx = 1;
z0 = [287;5;-176;0;2;0];

[Aeq_test1, beq_test1] = eq_cons(idx,A,B,z0,npred);

%% Functions
function [Aeq,beq]=eq_cons(idx,A,B,z0,npred)

% build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
% in the form Aeq*z=beq
% initial_idx specifies the time index of initial condition from the reference trajectory 
% A and B are function handles above

% Initialize
Aeq = zeros((1+npred)*6, 6*(npred+1) + 2*(npred));
beq = zeros((1+npred)*6,1);

% Add initial conditions to beq, rest are zeros
beq(1:6) = z0;

cAr = 1; cAc = 0; cB = 3*(npred+1);

for i = idx:idx+npred
    % Calculate matrices
    A_temp = A(i); B_temp = B(i);  
    % Initial conditions
    if i == idx;
        Aeq(1:3,1:3) = eye(3);
    else
        Aeq(3*cAr+1:3*cAr+3,3*cAc+1:3*cAc+3) = A_temp;
        Aeq(3*cAr+1:3*cAr+3,3*cAc+4:3*cAc+6) = -eye(3);
        Aeq(3*cAr+1:3*cAr+3,cB+1:cB+2) = B_temp;
        % Increment counters
        cAr = cAr + 1; cAc = cAc + 1; cB = cB + 2;
    end
   
end

end

function [Lb,Ub]=bound_cons(idx, U_ref, in_range, npred)
% initial_idx is the index along uref the initial condition is at

% Set the states upper/lower bound
Lb = -inf*ones(3*(npred+1),1);
Ub = inf*ones(3*(npred+1),1);

count = length(Lb)+1;

for i = idx:idx+npred-1
    % u
    Lb(count) = in_range(1,1) - U_ref(1,i);
    Ub(count) = in_range(1,2) - U_ref(1,i);
    d
    Lb(count+1) = in_range(2,1) - U_ref(2,i);
    Ub(count+1) = in_range(2,2) - U_ref(2,i);
     
    count = count+2;
end


end