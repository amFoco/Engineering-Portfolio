%% Brakes Calculator
%{
The purpose of this script is to expedite the braking math pertaining to:
weight per axle during a braking event, max deceleration, required braking
torque, pedal ratio, 
%}
%% Variable Definition

%{
- W_braked = weight on front axle during a stop of (a/g) deceleration
- W = weight on axle with no braking but with any aero effects
(download, drag) at the speed of interest
- W = gross weight of the vehilce
- decel = deceleration of the vehicle in "g" units; values can range from
over 3.0g for high speed with aero download to about 0.1g for ice
- h = height of the vehicle center of gravity above ground
- l = wheelbase
- W_dist = weight distribuition front to rear
- p = brake line pressure, lb/in^2
- F_x = Required longitudinal force from a wheel, lb
- R_l = Loaded radius of the tire
- r = brake caliper radius
- A_c = Total caliper piston area, in^2 (sum of all the pistons' areas)
- u_pad = CoF of brake pad
- R_pedal
%}

%% Braking Force per Axle
W = 578;
W_dist = 0.4;
W1 = W * W_dist;
W2 = W - W1;

V = 25*5280/3600; % mi/hr * 5280ft/mi * 1hr/3600sec
stop_dist = 20; % ft

h = 20/12;
l = 60/12;
decel = V^2/(2*stop_dist)/32.2;
W1_braked = W1 + W*decel*(h/l);
W2_braked = W2 - W*decel*(h/l);

%% Pressure required to stop the car
u_tire = 0.75;  % Coefficient of Friction to ground
r_tire = 11/12;
F_zF = W1_braked;
F_zR = W2_braked;
T_brakingF_Req = r_tire * F_zF * u_tire
T_brakingR_Req = r_tire * F_zR * u_tire

F_driver = 70;
r_rotorF = 8;
r_rotorR = 8;
no_piston = 2;  % two pistons per caliper
A_C1 = 1.5;
A_C2 = 1.5;
u_pad = 0.35;
BB_f = 0.63;
BB_r = 0.37;

bore = [13/16, 15/16, 3/4, 5/8, 7/10, 7/8, 1, 1/2];
A_MCF = (bore/2).^2 * pi;

PR = linspace(2, 5, 7);

for x = 1:length(PR)
    new_T_F(x,:) = (F_driver * PR(x) * BB_f * 2 * r_rotorF * u_pad * A_C1)./A_MCF./12;
    new_T_R(x,:) = (F_driver * PR(x) * BB_r * 2 * r_rotorR * u_pad * A_C2)./A_MCF./12;
end
disp(T_brakingF_Req*1.05)
new_T_F
disp(T_brakingR_Req*1.05)
new_T_R

% PR = 3.5;
% new_T_F = (F_driver * PR * BB_f * 2 * r_rotorF * u_pad * A_C1)./A_MCF./12;
% new_T_R = (F_driver * PR * BB_r * 2 * r_rotorR * u_pad * A_C2)./A_MCF./12;

%%
%{
Loop through bore of master cylinder, rotor size front and rear, piston
information, and pedal ratio.
[MC, PR, BB, Calipers]
%}
% BB_f = linspace(0.50, 0.70, 20);
% calipers = [2, 2.46, 4.68-3.13; 2, 1.38, 5.84-3.48];
% T_brakingF_Calc = zeros([size(A_MCF,2),size(PR,2),size(BB_f,2), size(calipers,1)]);
% T_brakingR_Calc = zeros([size(A_MCF),size(PR),size(BB_f), size(calipers,1)]);
% for ind_bore = 1:size(A_MCF,2)
%     for ind_PR = 1:size(PR)
%         for ind_BB = 1:size(BB_f,2)
%             for ind_calipers = 1:size(calipers, 1)
%                 T_brakingF_Calc(ind_bore, ind_PR, ind_BB, ind_calipers) = ...
%                     (F_driver * PR(ind_PR) * BB_f(ind_BB) * 2 * r_rotorF * u_pad * calipers(ind_calipers, 2))/A_MCF(ind_bore)/12;
%                 T_brakingR_Calc(ind_bore, ind_PR, ind_BB, ind_calipers) = ...
%                     (F_driver * PR(ind_PR) * (BB_f(ind_BB)-1) * 2 * r_rotorR * u_pad * calipers(ind_calipers, 2))/A_MCF(ind_bore)/12;
%             end
%         end
%     end
% end
% find(T_brakingF_Calc>=(T_brakingF_Req * 1.2))
