%% Check toolbox installation
% verify that the RWTH - Mindstorms NXT toolbox is installed.
if verLessThan('RWTHMindstormsNXT', '3.00');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
        ,'version 3.00 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
        ,'and follow the installation instructions!'));
end%if


%% Clear and close
COM_CloseNXT all
clear all
close all


%% Open Bluetooth connetion
%h = COM_OpenNXT('bluetooth32.ini');
h =COM_OpenNXTEx('Any', '00165315C85B', 'bluetooth.ini');
COM_SetDefaultNXT(h);

%% Constants and so on
Ports = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel

%% Initialize motor-object for straight line movement:
mStraight                   = NXTMotor(Ports);
% next command since we are driving in SYNC-mode. This should not be
% necessary with correct default values, but at the moment, I have to set
% it manually,
mStraight.SpeedRegulation   = false;  % not for sync mode
mStraight.Power             = 60;
mStraight.TachoLimit        = 720;
mStraight.ActionAtTachoLimit = 'coast';

%% create Seperate motor models
mB                    = NXTMotor(Ports(1)); %choose which motor from port
mB.SpeedRegulation    = true;  % Regulates speed to match input power value
mB.ActionAtTachoLimit = 'coast';
mB.TachoLimit = 360;
mB.Power = -30; %negative sign means the motor will do a 360 degrees in the NEGATIVE direction

mC                    = NXTMotor(Ports(2));
mC.SpeedRegulation    = true;  % not for sync mode
mC.ActionAtTachoLimit = 'coast';
mC.TachoLimit = 360;
mC.Power = -30; %Positive sign means the motor will do a 360 degrees in the Positive direction

%%The code
    mStraight.SendToNXT();
    mStraight.WaitFor();
    
    pause(3);
    
    mB.SendToNXT();
    mB.WaitFor();
    
    mC.SendToNXT();
    mC.WaitFor();
    
    
%%Turn motors off
mStraight.Stop('off');
%% Close Bluetooth connection
COM_CloseNXT(h);
