%% Suspension Analysis
% Suspension Analysis for BSAE/FSAE teams at Wichita State University
% Developed by yours truly, Adam Six (the electrical engineer)

%{
- W_t 
}%

%% Varaible Definition
% Vehicle Characteristics
W_t = 578;  % total weight, lbs
l = 5;      % wheelbase, ft
W_f = 278;  % front weight, lbs
W_r = 300;  % rear weight, lbs
T = 1/12 * [50, 48]; % Track width [front, rear], ft
H = 8.7931/12;  % CG to roll axis
Z_f = 11.73/12;  % Front roll center height, ft
Z_r = 2.74/12;  % Rear roll center height, ft
K_t = 1000; % tire vertical rate, lb/in

% Springs & Anti-Roll Bar Charactersitics
T_s = 0;
K_phi_des = 0;
K_theta_B = 0;
ir = 1;
L = [0, 0];
I_B = [0, 0];
K_s = [0, 0];
K_phi_B = [0, 0];

% Environment Characteristics
V = 25; % velocity, mph
R = 25; % radius of steady turn, ft

%% Analysis
[gradient, K_r(1), K_r(2), K_phi_f, K_phi_r, W_prime, roll_angle, delta_f, delta_r] = ...
    Ride_and_Roll_Rates(W_t, W_f, W_r, T(1), T(2), l, H, Z_f, Z_r, R, V);
[K_w, K_phi_a, K_phi] = Wheel_Center_Rates(K_r, K_t, T, K_phi_des, T_s);

[K_s(1), K_phi_B(1)] = Installation_Ratios(K_w(1), ir, K_theta_B, L(1), I_B(1), T(1));
[K_s(2), K_phi_B(2)] = Installation_Ratios(K_w(2), ir, K_theta_B, L(2), I_B(2), T(2));
