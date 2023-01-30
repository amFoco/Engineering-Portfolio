%% Installation Ratios
%{
- Determines spring and anti-roll bar charactersitics for desired ride/roll
rates
- Installation ratio is a geometric concepts that relates the change in
length (or angle) of a force-producing device (spring, shock, anti-roll
bar) to a change in vertical wheel center movement

%}

%% Variable Definitions
%{
- K_w = approximate effective wheel rate, lb/in
- K_s = Spring Rate
- ir = the rate of change of spring compression with wheel movement (delta
spring/delta wheel)
- K_phi_B = contribution of the anti-roll bar rate to the total car roll
rate, lb-ft/deg
- K_theta_B = anti-roll bar angular (twist) rate, lb-ft/deg
- I_B = linear installation ratio of the anti-roll bar, inches of bar
attachment point movemnet per inch of wheel center movement, in/in
- T = track width, ft
- L = anti-roll bar lever arm, ft.
%}
function [K_s, K_phi_B] = Installation_Ratios(K_w, ir, K_theta_B, L, I_B, T)

K_s = K_w/(ir^2);   % spring rate, lb/in
K_phi_B = K_theta_B*L^2/(I_B^2*T^2);    % Anti roll bar rate, lb-ft/rad

end
