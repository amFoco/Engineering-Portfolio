%% Gear Drivetrain Calculator

%% Dependent Variables 
%{
- Engine = Baja SAE Restricted Kohler CH440 provided info
- T_size = tire radius size in feet
- W = gross vehicle weight
- W_dist = front weight distribution
- W_f = front axle weight
- W_r = rear axle weight

All distances are in feet and all weights are in pounds
- W_L = Payload weight
- W_S = Sled weight
- L_L = Height of hitch point on sled
- L_T = Lenghth of hitch point to axle on sled
- L_S = Length of hitch point to CG on sled
- L_N = Length of hitch point to support plate on sled
- L_F = Length of hitch point to CG of payload
- u = Coefficient of friction between sled  and surface
- theta = angle of cable in degress
%}
Engine.RPM = [2400 2600 2800 3000 3200 3400 3600];
Engine.Torque = [18.5 18.1 17.4 16.6 15.4 14.5 13.5];
Engine.Power = [8.5 9 9.3 9.5 9.4 9.4 9.3];
Tire_size = 11/12;
W = 578;
W_dist = 0.4;
W_f = W * W_dist;
W_r = W * (1-W_dist);

theta = 15;
W_L = 1000;   % lbs
W_S = 1000;  % lbs
L_L = 4;
L_T = 11;
L_S = 6;   % ft
L_N = 2;
L_F = 0.5;
u = 0.45;

%% Traction - Low Gear
%{ 
- F_C = Cable tension in lbs
- T_req = Torque required
- u_min = Coefficient of friction from tire to asphalt
- u_max = Coefficient of friction from tire to dire
- F_D_min = Minimum driving force in lbs
- F_D_max = Maximum driving force in lbs
- T_limit = Max theoretical torque
- GR_low = Lowest ratio required
Due to 4WD, F_D_min/max will include front and rear tires
%}

F_C = (-W_L * (L_L-L_T) + W_S*(L_T-L_S))/(L_T*(cosd(theta)/u) + ...
    L_T*sind(theta) - L_N*(cosd(theta)/u)-L_F*cosd(theta));
T_req = Tire_size * F_C * cosd(theta);

u_min = 0.788;
u_max = 1.183;
F_D_min = u_min * (W/2); 
F_D_max = u_max * (W/2);

T_limit = 2*F_D_max*Tire_size;
GR_low = T_limit/max(Engine.Torque);

%% Speed - High Gear
%{
- p_air = STD of air in slugs/ft^3
- A_frontal = frontal surface area (includes control arms, firewall,
chassis, tires) in ft^2
- P_max_theo = maximum power available to wheels theoretically lb-ft/sec
- P_max_act = maximum power available to wheels actual
- C_D = coefficient of drag
- C_RR = coefficient of rolling resistance
- F_drag = force of drag on the car
- F_RR = force of rolling resistance on the car
- F_driving = required driving force
- e = drivetrain efficiency
%}
p_air = 2.38 * 10^-3;
A_frontal = 14;
C_D = 1.08;
C_RR = 0.068;
% e = 0.84;
e = .97*.97*.931*.98;

Vmax = 40;
Vmax = Vmax * 5280/3600; % ft/sec
P_max_theo = e * max(Engine.Power) * 550;
f = @(V) (1/2) * p_air * A_frontal * C_D * V^3 + C_RR*W*V - P_max_theo;
f_prime = @(V) (3/2)*p_air*A_frontal*C_D*V^2 + C_RR*W;

error = 1;
itr = 0;
while itr < 5001
    V1 = Vmax - f(Vmax)/f_prime(Vmax);
    error = V1/Vmax;
    if error < 0
        error = Vmax/V1;
    end
    if (error <= 1 && error > 0.99 && f(Vmax) <= 0) || itr == 5000
        V1 = Vmax * 3600/5280;
        sprintf('The max speed the car will travel is %f mph', V1)
        break
    else
        Vmax = V1;
    end
    itr = itr+1;
end
V_rpm = (V1*63360)/(22*pi*60);
GR_high = 3600/(V_rpm);

%% Finding the High ratio for 40 mph
V_40_rpm = (40*5280/3600)*63360/(22*pi*60);
GR_40_high = 3600/(V_40_rpm)

%% Planetary Setup
