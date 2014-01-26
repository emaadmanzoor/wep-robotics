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
mStraight.Power             = 30;
mStraight.TachoLimit        = 720;
mStraight.ActionAtTachoLimit = 'coast';

%send command
mStraight.SendToNXT();

%% Initialize the data, figure and array x and time
data = mStraight.ReadFromNXT();
figure
hold on;
tic
x(1) = 0;
t(1) = 0;
m = 2;
 while(data.IsRunning)
     %update data in array
     t(m) = toc;
     x(m) = data.TachoCount;
     plot(t, x);
     drawnow;

     data = mStraight.ReadFromNXT(); % refresh
     m = m+1; %%increment array counter
 end%while   
 
 
%We close down our motors:
mStraight.Stop('off');


%% Close Bluetooth connection
COM_CloseNXT(h);