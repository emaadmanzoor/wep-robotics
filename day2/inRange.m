function newTheta = inRange(theta)

newTheta=mod(theta,2*pi);
if newTheta > pi
   newTheta=newTheta-2*pi;
end

end