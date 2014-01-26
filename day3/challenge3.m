%% Feedback Control
clc
clear all
COM_CloseNXT all
close all

%% Constants
l = 6;  % cm
r = 2.7; % cm

%% Open Bluetooth connetion
h = COM_OpenNXT('bluetooth.ini');
COM_SetDefaultNXT(h);

%% Actions
expectedDistance = 40; %cm
tolerance = 4; %cm

% Initialize sonar
SENSOR_PORT = 3;
OpenUltrasonic(SENSOR_PORT);

% Measure distance from wall
distance = GetUltrasonic(SENSOR_PORT);
m = 2;
plot(m, distance, 'b+');
hold on;

while(m < 2000)
    while abs(distance - expectedDistance) < tolerance
        goStraight(5, 20, r);
    end
    goRotate(sign(distance - expectedDistance) * -30, 20, r, l);
    goStraight(10, 20, r);
end