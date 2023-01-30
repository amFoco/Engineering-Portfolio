%% Wheel Center Rates

%% Variable Definition
function [K_w, K_phi_a, K_phi] = Wheel_Center_Rates(K_r, K_t, T, K_phi_des, T_s)

%K_w = [0, 0];    % Wheel center rate, lb/in
%K_r = [30.0609, 131.4174];    % Ride rate, lb/in
%K_t = 1000;    % tire vertical rate, lb/in - ASSUMPTION
%T = [50/12, 48/12];   % track width, ft

%K_phi_des = 0;  % Desired total roll rate, lb-ft/rad
%T_s = 350; 

%% Number Crunch
K_w = (K_r * K_t)./(K_t - K_r);

K_phi_a = (K_phi_des*(12*K_t.*T.^2/2))/(12*K_t.*(T.^2)/2-K_phi_des) - 12*K_w.*T.^2/2;
if T_s > 0
    K_phi = (12*(K_t.*T.^2/2)*12*(K_w.*T_s^2/2))/(12*(K_t.*T^.2/2)+12*(K_w.*T_s^2/2));
else
    K_phi = -1;
end

end
