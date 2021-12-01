clear
clc
tic
%% Vehicle Parameters
% Input limits 
delta_lim = [-0.5, 0.5];
Fx_lim = [-5000, 5000];

% Constants
m = 1400; Nw = 2; f = 0.01; Iz = 2667; a = 1.35; 
b = 1.45; By = 0.27; Cy = 1.2; Dy = 0.7; Ey = -1.6; 
Shy = 0; Svy = 0; g = 9.806; dt = 0.01;

% Reference trajectories
U_Ref = ones(2,17150);
Y_Ref = ones(6,17150);

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
delta = U_Ref(1,:);
u = Y_Ref(2,:); v = Y_Ref(4,:); r = Y_Ref(4,:);
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
A = @(i) eye(6) +  dt*[0, cos(Y_Ref(5,i)), 0, -sin(Y_Ref(5,i)), - Y_Ref(4,i)*cos(Y_Ref(5,i)) - Y_Ref(2,i)*sin(Y_Ref(5,i)),           0
                       0,               0, 0,       Y_Ref(6,i),                                                         0,  Y_Ref(4,i)
                       0, sin(Y_Ref(5,i)), 0,  cos(Y_Ref(5,i)),   Y_Ref(2,i)*cos(Y_Ref(5,i)) - Y_Ref(4,i)*sin(Y_Ref(5,i)),           0
                       0,     -Y_Ref(6,i), 0,                0,                                                         0, -Y_Ref(2,i)
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
Ndec = 6*(npred+1) + 2*(npred);

%% Testing Equality Constraints
idx = 1;
z0 = [287;5;-176;0;2;0];

[Aeq_test1, beq_test1] = eq_cons(idx,A,B,z0,npred);

%% Boundary Constraints
idx = 1;
[Lb_test1, Ub_test1] = bound_cons(idx, U_Ref, delta_lim, Fx_lim, npred);

%% Simulate

% Initialize
Q = eye(6);
Q(3,3) = 0.5;
R = [0.1 0; 0 0.01];

T = 0:dt:1;
Y_tilde = zeros(6,17150-npred);
Y_tilde(:,1) = [287;5;-176;0;2;0];
Y = zeros(6,17150-npred);
Y(:,1) = Y_tilde(:,1) + Y_Ref(:,1);

% Form H and f
H1 = kron(eye(11),Q); H2 = kron(eye(10),R); H3 = zeros(66,20);
H = [H1,H3; H3',H2];
f = zeros(Ndec,1);

% Loop through 
for i = 1:length(Y_Ref)-10
    % Redefine initial condition for given step
    Y0_tilde = Y_tilde(:,i);
    Y0 = Y(:,i);
    
    % Find constraints
    [Aeq, beq] = eq_cons(i, A, B, Y0_tilde, npred);
    [Lb, Ub] = bound_cons(i, U_Ref, delta_lim, Fx_lim, npred);
    
    % Optimize (no inequality constraints)
    zstar_tilde = quadprog( H, f, [], [], Aeq, beq, Lb, Ub);
    
    % Extract input + save for later plotting
    U_tilde(:,i) = zstar_tilde(34:35);
    U(:,i) = U_tilde(:,i) + U_Ref(:,i);
    
    % Simulate using ode45 (actual dynamics)
    [t, Ysim] = ode45(@(t,x)odefun(x,U(:,i)),T,Y0);
    Ysim = Ysim';
    Y(:,i+1) = Ysim(:,2);
    Y_tilde(:,i+1) = Y(:,i+1) - Y_Ref(:,i+1);
    
end
time = toc;
fprintf('Time: %0.3f\n',time)

%% Plotting
% subplot(3,1,1)
plot(Y_Ref(1,:),Y_Ref(3,:))
hold on
plot(Y(1,:),Y(3,:))
hold off
ylabel('y'); xlabel('x')
legend('Ref','Actual')

% subplot(3,1,2)
% plot(Y_Ref(1,:), U_Ref(1,:))
% hold on
% plot(Y(1,1:end-1), U(1,:))
% hold off
% ylabel('u'); xlabel('x')
% legend('Ref','Actual')

% subplot(3,1,3)
% plot(Y_Ref(1,:), U_Ref(2,:))
% hold on
% plot(Y(1,1:end-1), U(2,:))
% hold off
% ylabel('delta'); xlabel('x')
% legend('Ref','Actual')

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

cA = 1; cB = 6*(npred+1);

for i = idx:idx+npred
    % Calculate matrices
    A_temp = A(i); B_temp = B(i);  
    % Initial conditions
    if i == idx
        Aeq(1:6,1:6) = eye(6);
    else
        Aeq(6*cA + 1:6*cA + 6,6*(cA-1) + 1:6*(cA-1) + 6) = A_temp;
        Aeq(6*cA + 1:6*cA + 6,6*(cA-1) + 7:6*(cA-1) + 12) = -eye(6);
        Aeq(6*cA + 1:6*cA + 6,cB+1:cB+2) = B_temp;

        % Increment counters
        cA = cA + 1; cB = cB + 2;
    end
   
end

end

function [Lb,Ub]=bound_cons(idx, U_Ref, delta_lim, Fx_lim, npred)
% initial_idx is the index along uref the initial condition is at

% Set the states upper/lower bound
Lb = -inf*ones(6*(npred+1),1);
Ub = inf*ones(6*(npred+1),1);

count = length(Lb)+1;

for i = idx:idx+npred-1
   
    Lb(count) = delta_lim(1) - U_Ref(1,i);
    Ub(count) = delta_lim(2) - U_Ref(1,i);
    
    Lb(count+1) = Fx_lim(1) - U_Ref(2,i);
    Ub(count+1) = Fx_lim(2) - U_Ref(2,i);
     
    count = count+2;
end
end

function [dx] = odefun(x,u)
    % Constants
    m = 1400; Nw = 2; f = 0.01; Iz = 2667;
    a = 1.35; b = 1.45; By = 0.27; Cy = 1.2;
    Dy = 0.7; Ey = -1.6; Shy = 0; Svy = 0;
    g = 9.806; dt = 0.01;

    % Slip angles
    alpha_f = u(1) - atan(x(4) + a*x(6) ./ x(2));
    alpha_r = - atan(x(4) - b*x(6) ./ x(2));
    % 
    phi_yf = (1 - Ey) * (alpha_f + Shy) + (Ey/By)*atan(By*(alpha_f + Shy));
    phi_yr = (1 - Ey) * (alpha_r + Shy) + (Ey/By)*atan(By*(alpha_r + Shy));
    % Forces
    Fzf = (b / (a + b)) * m * g;
    Fyf = Fzf * Dy * sin(Cy*atan(By*phi_yf)) + Svy;
    Fzr = (a / (a + b)) * m * g;
    Fyr = Fzr * Dy * sin(Cy*atan(By*phi_yr)) + Svy;


    dx = [x(2)*cos(x(5)) - x(4)*sin(x(5));
          (1/m)*(-f*m*g + Nw*u(2) - Fyf*sin(u(1)) + x(4)*x(6));
          x(2)*sin(x(5)) + x(4)*cos(x(5));
          (1/m)*(Fyf*cos(u(1))+Fyr) - x(2)*x(6);
          x(6);
          (1/Iz)*(a*Fyf*cos(u(1)) - b*Fyr)];
        
end