%% Ride and Roll Rate Calculator
%{
This calculator (in it's current form) shows the ride and roll frequency on
an existing vehicle. The vehicle currently within this calculator is SRB-22
(also known as Bertha). A later iteration will map the dynamic change.
This tool will serve as a way to quickly choose hard characteristics on the
design of the vehicle.
%}
%% Possible Improvements
%{
- Store everything into a matrix for easier display
- Nested loops to iterate every possible change (allowed travel, roll
center heights, radius, velocity)
- For nested loops, store the best value
%}
%% Clean
% clear all
% clc
%% Variable Definition
function [gradient, K_rf, K_rr, K_phi_f, K_phi_r, W_prime, roll_angle, delta_f, delta_r] = ...
    Ride_and_Roll_Rates(W_t, W_f, W_r, T_f, T_r, l, H, Z_f, Z_r, R, V)

% roll_gradient = 1.5;    % deg/in

% W_t = 578;      % total weight, lbs
% W_f = 207.9;    % front weight, lbs
% W_r = 369.6;    % rear weight, lbs
% 
% T_f = 50/12;    % front track width, ft
% T_r = 48/12;    % rear track width, ft
% l = 60/12;      % wheelbase, ft
% 
% h = 20/12;      % CG height, in
% H = 8.914/12;   % CG to roll axis, in
% Z_f = 11.73/12; % front roll center height, ft
% Z_r = 2.74/12;  % rear roll center height, ft

% drive torque and longitudinal acceleration
T_d = 0;
A_x = 0;

% cornering conditions
alpha = 0;  % bank angle, deg
% R = 25;     % radius of turn, ft
% V = 25;     % velocity, mph

% first assumption
K_phi_f = W_f*1.67*(180/pi); % lb-ft/rad
K_phi_r = W_r*1.67*(180/pi); % lb-ft/rad


%% Calculate CG position
b = W_f*l/W_t; % feet
a = l-b; % feet

%% Lateral Acceleration
V = V*5280/3600; % mi/hr to ft/sec
A_alpha = (V^2)/(32.2*(-R)); % horizontal lateral acceleration, g's
A_y = A_alpha*cosd(alpha) - sind(alpha); % Lateral acceleration in car axis system, g's

sprintf("Lateral Acceleration: %f g's", A_y)

%% Effective Weight of the Car
W_prime = (W_t*(A_alpha*sind(alpha) + cos(alpha)))/l * [b, a];

% W_prime = W_t*(A_alpha*sind(alpha) + cos(alpha)); % lbs
% W_prime_f = W_prime*b/l;
% W_prime_r = W_prime*a/l;

%% Roll Gradient - The spot to start iterating
%phi = -W_t * H;
new_gradient = 0;
while abs(new_gradient*180/pi) < 1.5 || abs(new_gradient*180/pi) > 2.5
    gradient = (-W_t*H)/(K_phi_r + K_phi_f)*(180/pi); % deg/g
    
    sprintf("Roll Gradient: %f", gradient)
    roll_angle = A_y*gradient;
    sprintf("Roll Angle: %f", roll_angle)
    
    %% Weight Change
    W_change = zeros(1,2);
    W_change(1) = A_y * W_t/T_f * ((H*K_phi_f)/(K_phi_f+K_phi_r) + b/l * Z_f);
    W_change(2) = A_y * W_t/T_r * ((H*K_phi_r)/(K_phi_r+K_phi_f) + b/l * Z_r);

    W_dynamic = [W_f/2 - W_change(1), W_f/2 + W_change(1);...
        W_r/2 - W_change(2), W_r/2 + W_change(2)]; % [fo, fi; ro, ri]
    W_static = [W_dynamic(1, 1) - W_f/2, W_dynamic(1, 2) + W_f/2;...
        W_dynamic(2, 1) - W_r/2, W_dynamic(2, 2) + W_r/2]; % [fo, fi; ro, ri];

    W_f_change = A_y * W_t/T_f * ((H*K_phi_f)/(K_phi_f+K_phi_r) + b/l * Z_f);
    W_r_change = A_y * W_t/T_r * ((H*K_phi_r)/(K_phi_r+K_phi_f) + b/l * Z_r);
    
%     W_fo_1 = W_f/2 - W_f_change;
%     W_fi_1 = W_f/2 + W_f_change;
%     W_ro_1 = W_r/2 - W_r_change;
%     W_ri_1 = W_r/2 + W_r_change;
%     
%     W_fo_2 = W_fo_1 - W_f/2;
%     W_fi_2 = W_fi_1 + W_f/2;
%     W_ro_2 = W_ro_1 - W_r/2;
%     W_ri_2 = W_ri_1 + W_r/2;
    %% 
    Allowed_travel = 3.5; % inches
    K_rf = W_static(1, 1)/Allowed_travel; % front outside
    K_rr = W_static(2, 2)/Allowed_travel; % rear outside

%     K_rf = W_fo_2/Allowed_travel;
%     K_rr = W_ro_2/Allowed_travel;
    
    omega_f = 1/(2*pi) * sqrt((K_rf*12*32.2)/(W_f/2));
    omega_r = 1/(2*pi) * sqrt((K_rr*12*32.2)/(W_r/2));
    
    %% Modifications
    if omega_f < omega_r
        while omega_f < omega_r
            omega_f = ((omega_r*1.01)/omega_f)^2;
            K_rf = omega_f * K_rf;
        end
    elseif omega_f > omega_r
        while omega_f > omega_r
            omega_r = ((omega_f*1.01)/omega_r)^2;
            K_rr = omega_r * K_rr;
        end
    end
    %% New
    K_phi_f = 12*K_rf*T_f^2/2;
    K_phi_r = 12*K_rr*T_r^2/2;
    
    new_gradient = (-W_t * H)/(K_phi_r + K_phi_f);
    
    %%
    sprintf("New Roll Gradient: %f", new_gradient*180/pi)
end
delta_f = W_static(1,1)/K_rf;
delta_r = W_static(2, 1)/K_rr;

end