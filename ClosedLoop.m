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

mB = NXTMotor(Ports(1));
mB.SpeedRegulation = true; % not for sync mode
mB.ActionAtTachoLimit = 'coast';
mB.TachoLimit = 0;

mC = NXTMotor(Ports(2));
mC.SpeedRegulation = true; % not for sync mode
mC.ActionAtTachoLimit = 'coast';
mC.TachoLimit = 0;

%%make initial readings
Encoder_B_b = NXT_GetOutputState(MOTOR_B);
Encoder_C_b = NXT_GetOutputState(MOTOR_C);

% Wheel constants
l = 6;
r = 2.7;

% Control constants
k_a = 3;
k_b = 8;
k_rho = -4.5;

% Initial position
xi = 40;
yi = 30;
thetai = deg2rad(30);

% Goal position
xg = 0;
yg = 0;
thetag = 0;

tolerance = 0.01;
angle_tolerance = 1;

%% Control loop

x = xi;
y = yi;
theta = thetai;

while ~(abs(x - xg) < tolerance && abs(y - yg) < tolerance...
       && (abs(theta - thetag) < angle_tolerance ||...
           abs(theta - pi - thetag) < angle_tolerance))
    x
    y
    dx = x - xg;
    dy = y - yg;

    rho   = sqrt(dx^2 + dy^2);
    alpha = -theta + atan2(dx,dy);
    beta  = -theta - alpha;

    w = k_a*alpha + k_b*beta;
    if (alpha > -pi/2) && (alpha <= pi/2)
       v = k_rho*rho;
    else
       v = -k_rho*rho;
    end

    xi_dot = [cos(theta) 0; sin(theta) 0; 0 1] * [v; w];
    rphi_dot = [1 0 l; 1 0 -l; 0 1 0] * R(theta) * xi_dot;
    phi_dot = rphi_dot ./ (2*pi*r);
    rpm = phi_dot(1:2) .* 60;
    
    %%
    power = rpm;
    [max_val, ind] = max(abs(power));
    if max_val > 100
     scale = 100 / max_val;
     power = scale * power;
    end
    power = floor(power ./ 4.0)
    %%
    
    % Send new speed to NXT
    mB.Power = power(1);
    mC.Power = power(2);
    mB.SendToNXT();
    mC.SendToNXT();

    Encoder_B_a = NXT_GetOutputState(MOTOR_B);
    Encoder_C_a = NXT_GetOutputState(MOTOR_C);
    deltaEncoder1 = Encoder_B_a.RotationCount - Encoder_B_b.RotationCount;
    deltaEncoder2 = Encoder_C_a.RotationCount - Encoder_C_b.RotationCount;

    deltaX = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaY = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaTheta = ((deltaEncoder1 - deltaEncoder2)*(2*pi*r/360)/(2*l));

    theta = theta + deltaTheta;
    theta = inRange(theta);
    x = x - deltaX * cos(theta);
    y = y - deltaY * sin(theta);
end