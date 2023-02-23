%% Ride and Roll Rate Calculator
% This calculator (in it's current form) shows the ride and roll frequency
% on an existing vehicle. The vehicle currently within this calculator is 
% SRB-22 (also known as Bertha). A later iteration will map the dynamic 
% change. This tool will serve as a way to quickly choose hard 
% characteristics on the design of the vehicle.

% As this calculator is for a mini-baja vehicle, I have chosen to not
% include the calcuations for a roll bar to help reduce the complexity of
% the car. Eventually, it will be added to the script as I would like to
% design a full vehilce at some point.

% See "Race Car Vehicle Dynamics" Chapter 16, Page 579, "Ride and Roll
% Rates" for more information.

%% Possible Improvements
%{
- Store everything into a matrix for easier display
- Nested loops to iterate every possible change (allowed travel, roll
center heights, radius, velocity)
- For nested loops, store the best value
%}
function [new_gradient, K_rf, K_rr, K_phi_f, K_phi_r, W_prime, roll_angle, delta_f, delta_r] = ...
    Ride_and_Roll_Rates(W_t, W_f, W_r, T_f, T_r, l, H, Z_f, Z_r, R, V, alpha)
%% Variable Definition
%{
- roll_gradient = deg/in
- W_t = total weight, lbs
- W_f = front weight, lbs
- W_r = rear weight, lbs
- T_f = front track width, ft
- T_r = rear track width, ft
- l = wheelbase, ft
- h = CG height from ground, in
- H = CG to roll axis, ft
- Z_f = front roll center height, ft
- Z_r = rear roll center height, ft
- T_d = driveing torque, lb-ft
- A_x = longitudinal acceleration

% cornering conditions
- alpha = bank angle, deg
- R = radius of turn, ft
- V = velocity, mph
%}
% first assumption
K_phi_f = W_f*2*(180/pi); % lb-ft/rad
K_phi_r = W_r*2*(180/pi); % lb-ft/rad


%% Calculate CG position
b = W_f*l/W_t; % feet
a = l-b; % feet

%% Lateral Acceleration
V = V*5280/3600; % mi/hr to ft/sec
A_alpha = (V^2)/(32.2*(-R)); % horizontal accel, g's
A_y = A_alpha*cosd(alpha)-sind(alpha); % Lateral accel in car axis system, g's

sprintf("Lateral Acceleration: %f g's", A_y)

%% Effective Weight of the Car
% W_prime is the effective wieght of the car in the turn due to banking
% [front, rear]
W_prime = (W_t*(A_alpha*sind(alpha) + cos(alpha)))/l * [b, a];

%% Roll Gradient - The spot to start iterating

new_gradient = 0; % Variable initialization
% The following loop calculates the roll gradient by first converting the
% rad/g to deg/g, and only stops when the roll gradient is between 1.5 and
% 2.5. The boundary roll gradients can be changed depending on the
% application, but as I am making a performance vehicle, I want the roll
% gradient to be within this boundary to design the vehicle for more of a
% racing performance setup instead of comfort or general performance.
counter = 0;
while (abs(new_gradient*180/pi) < 1.5 || abs(new_gradient*180/pi) > 2.5) && counter < 1000
    % Calculates the base gradient
    gradient = (-W_t*H)/(K_phi_r + K_phi_f)*(180/pi); % deg/g
    % Just displays the current iteration of the gradient
%     sprintf("Roll Gradient: %f", gradient)
%     sprintf("Roll Angle: %f", roll_angle)
    
    %% Weight Change
    W_change = zeros(1,2); % allocating space
    % W_change(1) is the weight change on the front axle
    W_change(1) = A_y * W_t/T_f * ((H*K_phi_f)/(K_phi_f+K_phi_r) + b/l * Z_f);
    % W_change(2) is the weight change on the rear axle
    W_change(2) = A_y * W_t/T_r * ((H*K_phi_r)/(K_phi_r+K_phi_f) + b/l * Z_r);
    
    % W_dynamic is the individual tire loads/weights [front outside, front
    % inside, rear outside, rear inside]
    W_dynamic = [W_f/2 - W_change(1), W_f/2 + W_change(1);...
        W_r/2 - W_change(2), W_r/2 + W_change(2)]; % [fo, fi; ro, ri]
    
    % W_static is the change in weight/load from the static loads measured
    % on level ground
    W_static = [W_dynamic(1, 1) - W_f/2, W_dynamic(1, 2) + W_f/2;...
        W_dynamic(2, 1) - W_r/2, W_dynamic(2, 2) + W_r/2]; % [fo, fi; ro, ri];

    %% Choosing Ride Rates
    % The next step is to choose ride rates that work with the wheel loads,
    % meaning the suspension will be stiff enought such that the outside
    % suspension doesn not bottom out. 

    Allowed_travel = 4; % allowed jounce travel, inches
    % K_rf/r are the front and rear suspension ride rates required for the
    % load change within the allowed jounce travel 
    K_rf = W_static(1, 1)/Allowed_travel; % front outside, lb/in
    K_rr = W_static(2, 2)/Allowed_travel; % rear outside, lb/in
    
    % omega_f and omega_r are the ride frequencies
    omega_f = 1/(2*pi) * sqrt((K_rf*12*32.2)/(W_f/2));
    omega_r = 1/(2*pi) * sqrt((K_rr*12*32.2)/(W_r/2));
    
    %% Modifications
    % This is really where the iteration bit comes into play. As I am
    % aiming for a specific roll gradient, I need to change the ride
    % frequency, which effect the roll rate. 

    
    if omega_f < omega_r
        while omega_f < omega_r
            ratio = ((omega_r*1.01)/omega_f)^2;
            omega_f = omega_r*1.01;
            K_rf = ratio * K_rf;
        end
    elseif omega_f > omega_r
        while omega_f > omega_r
            ratio = ((omega_f*1.01)/omega_r)^2;
            omega_r = omega_f*1.01;
            K_rr = ratio * K_rr;
        end
    end
    %% New Roll Rates
    % The k_phi terms are the updated front and rear roll rates that better
    % accomodate the design goal of a specific roll gradient.
    K_phi_f = 12*K_rf*T_f^2/2;
    K_phi_r = 12*K_rr*T_r^2/2;
    
    new_gradient = (-W_t * H)/(K_phi_r + K_phi_f);
    roll_angle = A_y*new_gradient;
    %% Display Results
    sprintf("New Roll Gradient: %f", new_gradient*180/pi)
    sprintf("New Roll Angle: %f", roll_angle)
    counter = counter + 1;
end
new_gradient = new_gradient * 180/pi;
roll_angle = roll_angle * 180/pi;

%% Wheel Ride Displacements
% delta indicates a wheel posistion, the displacement from static condition
delta_f = W_static(1,1)/K_rf;
delta_r = W_static(2, 1)/K_rr;

end