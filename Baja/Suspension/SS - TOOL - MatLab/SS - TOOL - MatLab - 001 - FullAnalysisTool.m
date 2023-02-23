%% Suspension Analysis
% Suspension Analysis for BSAE/FSAE teams at Wichita State University
% Developed by yours truly, Adam Six (the electrical engineer)


%% Varaible Definition
%{
W_t = gross vehicle weight, lbs
dist = front weight distribution
W_f = static front axle weight
W_r = static rear axle weight
l = wheelbase, ft
T = trackwidth array (front and rear), ft
H = CG to roll axis vertical distance
Z_f = front roll center height
Z_r = rear roll center height
K_t = tire vertical spring rate, lb/in
T_s
%}

%% Vehicle Characteristics
% *NOTE* Anything multiplied by 1/12 is an inch to ft conversion

W_t = 575;  % total weight, lbs
l = 5;      % wheelbase, ft
dist = 0.45;    % front weight distribution
W_f = W_t * dist;
W_r = W_t - W_f;
T = 1/12 * [48, 45];
H = 5.04/12; 
Z_f = 5.0929/12;
Z_r = 6.88/12;
K_t = 500; % tire vertical rate, lb/in

%% Springs & Anti-Roll Bar Charactersitics
T_s = 0;
K_phi_des = 0;
K_theta_B = 0;
ir = 1;
L = [0, 0];
I_B = [0, 0];
K_s = [0, 0];
K_phi_B = [0, 0];

%% Environment Characteristics
V = 25; % velocity, mph
R = 25; % radius of steady turn, ft
alpha = 0; % bank in turn, deg

%% Analysis
[gradient, K_r(1), K_r(2), K_phi_f, K_phi_r, W_prime, roll_angle, delta_f, delta_r] = ...
    Ride_and_Roll_Rates(W_t, W_f, W_r, T(1), T(2), l, H, Z_f, Z_r, R, V, alpha);
[K_w, K_phi_a, K_phi] = Wheel_Center_Rates(K_r, K_t, T, K_phi_des, T_s);

[K_s(1), K_phi_B(1)] = Installation_Ratios(K_w(1), ir, K_theta_B, L(1), I_B(1), T(1));
[K_s(2), K_phi_B(2)] = Installation_Ratios(K_w(2), ir, K_theta_B, L(2), I_B(2), T(2));
