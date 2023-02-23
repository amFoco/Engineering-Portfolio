%% Wheel Center Rates
% This function is to convert the ride and roll rates of the car to wheel
% center rates. This conversion helps one determine the necessary spring
% rates for the car. K_phi_A is the wheel center rate, which can be
% reffered to the suspension component rate (spring/anti-roll bars)

% See "Race Car Vehicle Dynamics" chapter 16.2, page 591, "Transferring
% Ride/Roll Rates to Wheel Center Rates" for more information.

function [K_w, K_phi_A, K_phi] = Wheel_Center_Rates(K_r, K_t, T, K_phi_des, T_s)
%% Variable Definition
%{
K_w = Wheel center rate [front, rear], lb/in
K_r = Ride rate [front, rear], lb/in
K_t = tire vertical sprint rate, lb/in 
T = track width [front, rear], ft

K_phi_des =  Desired total roll rate, lb-ft/rad
K_phi_A = additional roll rate needed, lb-ft/rad

K_phi = roll rate, lb-ft/rad
T_s = spring track, ft (used for leaf springed axles)
%}
%% Number Crunch
K_w = (K_r * K_t)./(K_t - K_r);
% vertical axle rate = (ride rate * tire rate)/(tire rate - ride rate)
if T_s <= 0
    K_phi_A = (K_phi_des*(12*K_t.*T.^2/2))/(12*K_t.*(T.^2)/2-K_phi_des) - 12*K_w.*T.^2/2;
    K_phi = NaN;
else
    K_phi = (12*(K_t.*T.^2/2)*12*(K_w.*T_s^2/2))/(12*(K_t.*T^.2/2)+12*(K_w.*T_s^2/2));
end
return
end