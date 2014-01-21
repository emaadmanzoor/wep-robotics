%% Feedback Control
clc
clear all
COM_CloseNXT all
close all

%% initial and final position
poseInitial = [-60 ; -60 ;  pi/4];  %x, y, theta
poseFinal = [0 ; 0 ; 0];

%% Constants
l = 6;  % cm
r = 2.7; % cm

%% Check toolbox installation
% verify that the RWTH - Mindstorms NXT toolbox is installed.
if verLessThan('RWTHMindstormsNXT', '3.00');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
        ,'version 3.00 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
        ,'and follow the installation instructions!'));
end%if

%% inital deltaX, deltaY, deltaTheta are equal to poseInitial
x = poseFinal(1) - poseInitial(1);
y = poseFinal(2) - poseInitial(2);
theta = poseInitial(3);

% controller constants
k = [3 8 -1.5*3]; %rho,alpha,beta

%% Time increment for simulator
inc = 0.01;

%% Open Bluetooth connetion
h = COM_OpenNXT('bluetooth.ini'); %for 32 bit
%h =COM_OpenNXTEx('Any', '00165315C85B', 'bluetooth.ini');
COM_SetDefaultNXT(h);

%% create wheel models
Ports = [MOTOR_B; MOTOR_C];
mB                    = NXTMotor(Ports(1));
mB.SpeedRegulation    = true;  % not for sync mode
mB.ActionAtTachoLimit = 'coast';
mB.TachoLimit = 0;

mC                    = NXTMotor(Ports(2));
mC.SpeedRegulation    = true;  % not for sync mode
mC.ActionAtTachoLimit = 'coast';
mC.TachoLimit = 0;

%%make initial readings
Encoder_B_b = NXT_GetOutputState(MOTOR_B);
Encoder_C_b = NXT_GetOutputState(MOTOR_C);

%%Main Loop
figure
m = 2;
targetReached = false;
while m<2000 && ~targetReached
    
    %% CONTROLLER: compute rho, alpha
    rho = sqrt(x^2 + y^2);
    alpha = -theta + atan2(y,x);
    alpha = inRange(alpha); % force alpha to be between -pi and pi
    
    %compute beta, v, w based on range that alpha belongs to
    % to allow forward and backward movement
    if (alpha>-pi/2 && alpha<=pi/2 )
        beta= -theta - alpha;
        beta = inRange(beta);
        v_w = [k(1)*rho ;k(2)*alpha+k(3)*beta];
    else
        alpha=inRange(atan2(y,x)-theta +pi);
        beta= -atan2(y,x);
        beta = inRange(beta);
        v_w = [-k(1)*rho ;k(2)*alpha+k(3)*beta];
    end
    
    %% DYNAMIC MODEL, convert v and w to phidot
    %compute kiI_dot
    kiI_dot = [cos(theta)*v_w(1) ; sin(theta)*v_w(1); v_w(2)];%xdot, ydot, thetadot
    
    %compute wheel speeds
    J1 = [1 0 l ; 1 0 -l ; 0 1 0];
    Rtheta = [cos(theta) sin(theta) 0 ; -sin(theta) cos(theta) 0 ; 0 0 1];
    rphidot = J1*Rtheta*kiI_dot;
    
    %convert speed to power
    %phi_dot is in rad/s, convert it to revs per min
    phidot = (rphidot./(2*pi*r));  %cm/s to RPS
    rpm = phidot .* (60 );
    %convert rpm to power (using: http://www.philohome.com/nxtmotor/nxtmotor.htm, 1st graph)
    power = (rpm ./ (10))
    
    %% SEND POWER VALUES TO MOTOR
    % make sure speed in range -100,100 and is an integer
    Pmax=40;
    Pmin=4;
    % Scale power down
    power = power .* Pmax/100;
    
    if(power(1) > Pmax)
        power(1) = Pmax;
    elseif(power(1) < - Pmax)
        power(1) = -Pmax;
    end
    if(power(2) > Pmax)
        power(2) = Pmax;
    elseif(power(2) < - Pmax)
        power(2) = -Pmax;
    end
    if(power(1) < Pmin && power(1) > 0)
        power(1) = Pmin;
    elseif(power(1) > -Pmin && power(1) < 0)
        power(1) = -Pmin;
    end
    if(power(2) < Pmin && power(2) > 0)
        power(2) = Pmin;
    elseif(power(2) > -Pmin && power(2) < 0)
        power(2) = -Pmin;
    end
    
    %send new speeds to NXT
    mB.Power              = floor(power(1));
    mC.Power              = floor(power(2));
    mB.SendToNXT();
    mC.SendToNXT();
    
    %% GET FEEDBACK
    Encoder_B_a = NXT_GetOutputState(MOTOR_B);
    Encoder_C_a = NXT_GetOutputState(MOTOR_C);
    % Change in encoders since last loop
    deltaEncoder1 = Encoder_B_a.RotationCount - Encoder_B_b.RotationCount;
    deltaEncoder2 = Encoder_C_a.RotationCount - Encoder_C_b.RotationCount;    
    %Save encoder values for next loop
    Encoder_B_b = Encoder_B_a;
    Encoder_C_b = Encoder_C_a;
    
    % compute new change in x y and theta
    deltaX = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaY = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaTheta = ((deltaEncoder1-deltaEncoder2)*(2*pi*r/360)/(2*l));
    
    % compute new pose x,y,theta
    % never forget that x = final - current position, and a positive change in current
    % position is a NEGATIVE change in delta x
    theta = theta + deltaTheta;
    theta = inRange(theta) %force theta to belong to [-pi,pi]
    x = x - deltaX*cos(theta);
    y = y - deltaY*sin(theta);
    
    %% Plot the point now
    plot(-x,-y)
    drawnow
    hold on

    %% TARGET REACHED?
    tolerance = 1; %%in cm
    angle_tolerance = .1;  %in rad
    % check if error of x and y less than tolerance
    % and if theta or (theta - pi) (don't allow robot to rotate by pi) is less than angle_tolerance
    if((abs(x) < tolerance && abs(y) < tolerance && (abs(theta) < angle_tolerance || abs(theta-pi) < angle_tolerance) ))
        targetReached = true;
    else targetReached = false;
    end
    
    m=m+1;
end

%% Stop the motors
mB.Stop('off');
mC.Stop('off');

%% Close connection with robot
COM_CloseNXT(h);