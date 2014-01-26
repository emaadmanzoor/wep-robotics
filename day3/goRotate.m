function goRotate( theta, power, r, l )
    % With the robot facing north (up), this makes it rotate
    % counterclockwise (if theta is positive) or clockwise
    % (if theta is negative)
    numDegrees = rad2deg(l * deg2rad(theta) / r)
    numDegrees = floor(numDegrees);
    
    direction = sign(numDegrees);
    numDegrees = abs(numDegrees);
    
    Ports = [MOTOR_B; MOTOR_C];
    
    mB                    = NXTMotor(Ports(1));
    mB.SpeedRegulation    = true;  % not for sync mode
    mB.ActionAtTachoLimit = 'brake';
    mB.Power = power * direction;
    mB.TachoLimit = numDegrees;

    mC                    = NXTMotor(Ports(2));
    mC.SpeedRegulation    = true;  % not for sync mode
    mC.ActionAtTachoLimit = 'brake';
    mC.Power = -power * direction;
    mC.TachoLimit = numDegrees;

    mB.SendToNXT();
    mC.SendToNXT();
    
    mB.WaitFor();
    mC.WaitFor();
end