function goStraight( d, power, r )
    numDegrees = rad2deg(d/r);
    
    Ports = [MOTOR_B; MOTOR_C];
    mStraight = NXTMotor(Ports);
    mStraight.SpeedRegulation = false;
    mStraight.Power = power;
    mStraight.ActionAtTachoLimit = 'brake';
    mStraight.TachoLimit = floor(numDegrees);
    mStraight.SendToNXT();
    mStraight.WaitFor();
end