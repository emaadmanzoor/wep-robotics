%% Check toolbox installation
% verify that the RWTH - Mindstorms NXT toolbox is installed.
if verLessThan('RWTHMindstormsNXT', '3.00');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
        ,'version 3.00 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
        ,'and follow the installation instructions!'));
end%if

%% Feedback Control
clear all
COM_CloseNXT all
close all
clc

%% Open Bluetooth connetion
h =COM_OpenNXT('bluetooth.ini'); clc;
COM_SetDefaultNXT(h);

%% Initial and final position
poseInitial = [-60 ; 0 ;  pi/2];  %x, y, theta
poseFinal = [0 ; 0 ; 0];

%% Constants
l = 6;   % cm; distance between the wheels
r = 2.7; % cm; wheel radius

% Rotational alphas
a1 = 0.01;
a2 = 0.01;

% Translational alphas
a3 = 0.01;
a4 = 0.01;

% controller constants
k = [3 8 -1.5*3]; % k_rho, k_alpha, k_beta

% Time increment for simulator
inc = 0.01;

% Tolerances
tolerance = 1;         % in cm
angle_tolerance = .02;  % in rad

%% Inital deltaX, deltaY, deltaTheta are equal to poseInitial
x(1)     = poseFinal(1) - poseInitial(1);
y(1)     = poseFinal(2) - poseInitial(2);
theta(1) = poseFinal(3) - poseInitial(3);

x(2:100)     = x(1);
y(2:100)     = y(1);
theta(2:100) = theta(1);

%% Create wheel models and make initial readings
Ports = [MOTOR_B; MOTOR_C];
mB                    = NXTMotor(Ports(1));
mB.SpeedRegulation    = true;  % not for sync mode
mB.ActionAtTachoLimit = 'coast';
mB.TachoLimit = 0;

mC                    = NXTMotor(Ports(2));
mC.SpeedRegulation    = true;  % not for sync mode
mC.ActionAtTachoLimit = 'coast';
mC.TachoLimit = 0;

Encoder_B_prev = NXT_GetOutputState(MOTOR_B);
Encoder_C_prev = NXT_GetOutputState(MOTOR_C);

%% Main Loop
figure
m = 2;
targetReached = false;
while m<2000 && ~targetReached
    m=m+1;
    
    % CONTROLLER: compute rho, alpha
    rho = sqrt(x(1)^2 + y(1)^2);
    alpha = -theta(1) + atan2(y(1),x(1));
    alpha = inRange(alpha); % force alpha to be between -pi and pi
    
    %compute beta, v, w based on range that alpha belongs to
    % to allow forward and backward movement
    if (alpha>-pi/2 && alpha<=pi/2 )
        beta= -theta(1) - alpha;
        beta = inRange(beta);
        v_w = [k(1)*rho; k(2)*alpha+k(3)*beta];
    else
        alpha=inRange(atan2(y(1),x(1))-theta(1) +pi);
        beta= -atan2(y(1),x(1));
        beta = inRange(beta);
        v_w = [-k(1)*rho; k(2)*alpha+k(3)*beta];
    end
    
    % DYNAMIC MODEL, convert v and w to phidot
    % compute kiI_dot
    kiI_dot = [cos(theta(1))*v_w(1) ; sin(theta(1))*v_w(1); v_w(2)];%xdot, ydot, thetadot
    
    % compute wheel speeds
    J1 = [1 0 l ; 1 0 -l ; 0 1 0];
    Rtheta = [cos(theta(1)) sin(theta(1)) 0 ; -sin(theta(1)) cos(theta(1)) 0 ; 0 0 1];
    rphidot = J1*Rtheta*kiI_dot;
    
    % convert speed to power
    % phi_dot is in rad/s, convert it to revs per min
    phidot = (rphidot./(2*pi*r));  %cm/s to RPS
    rpm = phidot .* (60 );
    % convert rpm to power (using: http://www.philohome.com/nxtmotor/nxtmotor.htm, 1st graph)
    power = (rpm ./ 10)
    
    % SEND POWER VALUES TO MOTOR
    % make sure speed in range -100,100 and is an integer
    Pmax=40;
    Pmin=0;
    % Scale power down
    max_pow = max(abs(power));
    if max_pow > Pmax
        scale = Pmax / max_pow;
        power = power .* scale;
    end
    pow_sign = sign(power);
    power = floor(abs(power)) .* pow_sign;
    
    %send new speeds to NXT
    mB.Power = power(1);
    mC.Power = power(2);
    mB.SendToNXT();
    mC.SendToNXT();
    
    % GET FEEDBACK
    Encoder_B = NXT_GetOutputState(MOTOR_B);
    Encoder_C = NXT_GetOutputState(MOTOR_C);
    % Change in encoders since last loop
    deltaEncoder1 = Encoder_B.RotationCount - Encoder_B_prev.RotationCount;
    deltaEncoder2 = Encoder_C.RotationCount - Encoder_C_prev.RotationCount;    
    %Save encoder values for next loop
    Encoder_B_prev = Encoder_B;
    Encoder_C_prev = Encoder_C;
    
    % compute new change in x(1) y(1) and theta(1)
    deltaX = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaY = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaTheta = ((deltaEncoder1-deltaEncoder2)*(2*pi*r/360)/(2*l));
    
    % compute new pose x(1),y(1),theta(1)
    % never forget that x(1) = final - current position, and a positive change in current
    % position is a NEGATIVE change in delta x(1)
    theta(1) = theta(1) + deltaTheta;
    theta(1) = inRange(theta(1)); %force theta(1) to belong to [-pi,pi]
    x(1) = x(1) - deltaX*cos(theta(1));
    y(1) = y(1) - deltaY*sin(theta(1));
    
    % Plot clouds of points
    % compute new pose x',y',theta', we still should not update x(1), y(1), theta(1)
    theta_prime = theta(1) + deltaTheta;
    theta_prime = inRange(theta_prime); %force theta to belong to [-pi,pi]    
    x_prime = x(1) - deltaX*cos(theta_prime);
    y_prime = y(1) - deltaY*sin(theta_prime);
    
    for i = 2:500
        dr1 = atan2(y_prime - y(1), x_prime - x(1)) - theta(1);
        dtr = sqrt((y(1) - y_prime)^2 + (x(1)- x_prime)^2);
        dr2 = theta_prime - theta(1) - dr1;
 
        dr1_h = dr1 - mysample(abs(a1*dr1 + a2*dtr));
        dtr_h = dtr - mysample(abs(a3*dtr + a4*(dr1+dr2)));
        dr2_h = dr2 - mysample(abs(a1*dr2 + a2*dtr));
       
        x(i) = x(i) + dtr_h*cos(inRange(theta(i)+dr1_h));
        y(i) = y(i) + dtr_h*sin(inRange(theta(i)+dr1_h));
        theta(i) = inRange(theta(i) + (dr1_h + dr2_h)); 
        
        plot(-x(i), -y (i))
    end
    
    % Plot the point now
    plot(-x(1),-y(1),'.r')
    drawnow
    hold on

    % TARGET REACHED?
    % check if error of x(1) and y(1) less than tolerance
    % and if theta or (theta - pi) (don't allow robot to rotate by pi) is less than angle_tolerance
    if((abs(x(1)) < tolerance && abs(y(1)) < tolerance && (abs(theta(1)) < angle_tolerance || abs(theta(1)-pi) < angle_tolerance) ))
        targetReached = true;
    else targetReached = false;
    end
end

%% Stop the motors
mB.Stop('off');
mC.Stop('off');

%% Close connection with robot
COM_CloseNXT(h);
