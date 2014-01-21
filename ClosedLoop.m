% Closed loop robot navigation

% Clear and close
COM_CloseNXT all
clear all
close all

% Open Bluetooth connetion
h = COM_OpenNXT('bluetooth.ini');
COM_SetDefaultNXT(h);

%% Initialization
Ports = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel
DrivingSpeed     = 60;
TurningSpeed     = 40;

mB = NXTMotor(Ports(1));
mB.SpeedRegulation = true; % not for sync mode
mB.ActionAtTachoLimit = 'coast';
mB.TachoLimit = 0;

mC = NXTMotor(Ports(2));
mC.SpeedRegulation = true; % not for sync mode
mC.ActionAtTachoLimit = 'coast';
mC.TachoLimit = 0;

%%make initial readings
%Encoder_B_b = NXT_GetOutputState(MOTOR_B);
%Encoder_C_b = NXT_GetOutputState(MOTOR_C);

% Wheel constants
l = 6;
r = 2.7;

% Control constants
k_a = 3;
k_b = 8;
k_rho = -4.5;

% Initial position
xi = [40 30 deg2rad(30)];

% Goal position
xi_g = [0 0 0];

tolerance = 0.001;
angle_tolerance = 1;

%% Control loop

%while abs(xi(1) - xi_g(1)) < tolerance && abs(xi(2) - xi_g(2)) < tolerance...
%      (abs(xi(3) - xi_g(3)) < angle_tolerance ||...
%       abs(xi(3) - pi - xi_g(3)) < angle_tolerance)

   dxi = xi - xi_g;
   theta = xi(3);
   
   rho   = sqrt(sum(dxi(1:2).^2));
   alpha = -theta + atan2(dxi(1),dxi(2));
   beta  = -theta - alpha;
   w = k_a*alpha + k_b*beta;
   
   if (alpha > -pi/2) && (alpha <= pi/2)
       v = k_rho*rho;
   else
       v = -k_rho*rho;
   end
   
   xi_dot = [cos(theta) 0; sin(theta) 0; 0 1]*[v; w];
   rphi_dot = [1 0 l; 1 0 -l; 0 1 0]*R(theta)*xi_dot;
   phi_dot = rphi_dot ./ (2*pi*r);
   rpm = phi_dot(1:2) .* 60;
   power = rpm;
   
   [max_val, ind] = max(power);
   if max_val > 100
     scale = 100 / max_val;
     power = scale * power
   end
   
   % Send new speed to NXT
   mB.Power = power(1)
   mC.Power = power(2)
   %mB.SendToNXT();
   %mC.SendToNXT();
%end

