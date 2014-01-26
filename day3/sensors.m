%% Feedback Control
clc
clear all
COM_CloseNXT all
close all

%% initial and final position
poseInitial = [-120 ; -60 ;  -3*pi/4];  %x, y(1), theta
poseFinal = [0 ; 0 ; 0];

%% Constants
l = 6;  % cm
r = 2.7; % cm


% Rotational alphas
a1 = 0.01;
a2 = 0.01;
% Translational alphas
a3 = 0.01;
a4 = 0.01;

%% Check toolbox installation
% verify that the RWTH - Mindstorms NXT toolbox is installed.
if verLessThan('RWTHMindstormsNXT', '3.00');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
        ,'version 3.00 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
        ,'and follow the installation instructions!'));
end%if

%% inital deltaX, deltaY, deltaTheta are equal to poseInitial
x(1) = poseFinal(1) - poseInitial(1);
y(1) = poseFinal(2) - poseInitial(2);
theta(1) = poseInitial(3);

x(2:100) = x(1);
y(2:100) = y(1);
theta(2:100) = theta(1);

% controller constants
k = [3 8 -1.5*3]; %rho,alpha,beta

%% Time increment for simulator
inc = 0.01;

%% Open Bluetooth connetion
%h = COM_OpenNXT('bluetooth32.ini'); %for 32 bit
h =COM_OpenNXTEx('Any', '00165315C85B', 'bluetooth.ini');
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

% Initialize ultrasonic sensor
SENSOR_PORT = 3;
OpenUltrasonic(SENSOR_PORT);
distanceToMaintain = 40; % cm
distanceError = 4; % +- 4cm

%%Main Loop
figure
m = 2;
targetReached = false;

%while m<2000 && ~targetReached
while m<2000

    % Get distance from wall
    distance = GetUltrasonic(SENSOR_PORT);
    

    %% CONTROLLER: compute rho, alpha
    rho = sqrt(x(1)^2 + y(1)^2);
    alpha = -theta(1) + atan2(y(1),x(1));
    alpha = inRange(alpha); % force alpha to be between -pi and pi
    
    %compute beta, v, w based on range that alpha belongs to
    % to allow forward and backward movement
    if (alpha>-pi/2 && alpha<=pi/2 )
        beta= -theta(1) - alpha;
        beta = inRange(beta);
        v_w = [k(1)*rho ;k(2)*alpha+k(3)*beta];
    else
        alpha=inRange(atan2(y(1),x(1))-theta(1) +pi);
        beta= -atan2(y(1),x(1));
        beta = inRange(beta);
        v_w = [-k(1)*rho ;k(2)*alpha+k(3)*beta];
    end
    
    %% DYNAMIC MODEL, convert v and w to phidot
    %compute kiI_dot
    kiI_dot = [cos(theta(1))*v_w(1) ; sin(theta(1))*v_w(1); v_w(2)];%xdot, ydot, thetadot
    
    %compute wheel speeds
    J1 = [1 0 l ; 1 0 -l ; 0 1 0];
    Rtheta = [cos(theta(1)) sin(theta(1)) 0 ; -sin(theta(1)) cos(theta(1)) 0 ; 0 0 1];
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
    
    % compute new change in x(1) y(1) and theta(1)
    deltaX = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaY = (0.5*(deltaEncoder1 + deltaEncoder2)*(2*pi*r/360));
    deltaTheta = ((deltaEncoder1-deltaEncoder2)*(2*pi*r/360)/(2*l));
    
    %% compute new pose x',y',theta', we still should not update x(1), y(1), theta(1)
    theta_prime = theta(1) + deltaTheta;
    theta_prime = inRange(theta_prime); %force theta to belong to [-pi,pi]    
    x_prime = x(1) - deltaX*cos(theta_prime);
    y_prime = y(1) - deltaY*sin(theta_prime);
    
    for i = 2:100
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
    
    % compute new pose x(1),y(1),theta(1)
    % never forget that x(1) = final - current position, and a positive change in current
    % position is a NEGATIVE change in delta x(1)
    theta(1) = theta(1) + deltaTheta;
    theta(1) = inRange(theta(1)) %force theta(1) to belong to [-pi,pi]
    x(1) = x(1) - deltaX*cos(theta(1));
    y(1) = y(1) - deltaY*sin(theta(1));
    
    %% Plot the point now
    plot(-x(1),-y(1),'.r')
    drawnow
    hold on

    %% TARGET REACHED?
    tolerance = 3; %%in cm
    angle_tolerance = .2;  %in rad
    % check if error of x(1) and y(1) less than tolerance
    % and if theta or (theta - pi) (don't allow robot to rotate by pi) is less than angle_tolerance
    if((abs(x(1)) < tolerance && abs(y(1)) < tolerance && (abs(theta(1)) < angle_tolerance || abs(theta(1)-pi) < angle_tolerance) ))
        targetReached = true;
    else targetReached = false;
    end
    
    m=m+1;
end

%% Stop the motors
mB.Stop('off');
mC.Stop('off');

%% Stop the ultrasonic sensor
CloseSensor(SENSOR_PORT);

%% Close connection with robot
COM_CloseNXT(h);
