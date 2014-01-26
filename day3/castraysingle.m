% Code to perform Raycasting on a Supplied Map to return simulated Lidar
% Range values.
% % Author- Shikhar Shrestha, IIT Bhubaneswar

% Input parameters are x-position, y-position, Binary image of map, number
% of rays to cast, Lidar range in cm.
% % Returned Range values are obtained on scanning clockwise. 
function [range]=castraysingle(xc,yc,theta, map,lidarrange)
% % Conversion factor from pixels to centimeters.
pixeltocm=1;
if map(ceil(yc),ceil(xc))==1
    range=0;
    return;
end

showfigure = 0;
if showfigure
figure(2);
hold off;
imshow(map);
hold on;
plot(xc,yc,'b*');
end
range=0;

% n = 1;
% theta=20;


r=linspace(0,lidarrange,1000);
y=yc+(r*sin(theta));
x=xc+(r*cos(theta));
% Removing points out of map
temp=[];
for k=1:numel(x)
    if x(k)>size(map,2) || y(k)>size(map,1) || x(k)<=0 || y(k)<=0
        temp=[temp;k];
    end
end
x(temp)=[];
y(temp)=[];

if showfigure
figure(2);
plot(x,y,'r');
end
% Computing Intersections
xint=round(x);
yint=round(y);
% Correcting zero map indexing
for l=1:numel(xint)
    if xint(l)==0
        xint(l)=1;
    end
    if yint(l)==0
        yint(l)=1;
    end
end

b=[];
for j=1:numel(xint)
b=[b;map(yint(j),xint(j))];
end
ind=find(b==1);

if ~isempty(ind)
xb=x(ind(1));
yb=y(ind(1));

if showfigure
figure(2);
plot(xb,yb,'g*');
end

dist=sqrt((xc-xb).^2 + (yc-yb).^2);
range=dist;  
end
%pause(0.000001);

% Converting to mm from pixels.
range=range/pixeltocm;%%divide by 10 to get in cm
% % % % % % % range=flipud(range);