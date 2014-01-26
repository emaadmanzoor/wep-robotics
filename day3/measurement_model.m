%%Measurement model function
%% execute these functions first
% clc
% clear all
% close all
% A = imread('mymap.jpg');%%each pixel = 1mm
% map_bw = im2bw(A,0.5);
% figure
% imshow(map_bw)
%castrays(200,100,map_bw,1,800)

%% Start of function
function [output]=measurement_model(xc, yc, sensor_angle, z,  map)

%%measurement model Constants
z_max = 200; %maximum distance that can be read using the sensor in MM

weight_hit = .95;
weight_short = .01;
weight_max = 0.02;
weight_rand = .02;

q = 1;
K=length(z);

%z = castrays(xc, yc, map, length(mean), 1000);

for k=1:K
mean = castraysingle(xc, yc, sensor_angle(k), map, z_max);
%% Probability of Hit
%Normalizing the gaussian in local measurement noise 
sigma_hit = 1;
%Normalizer = normcdf(z_max);

% for i = 0:.1:z_max
%     Normalizer = Normalizer+normpdf(i,mean,sigma_hit)*0.1; %% integration
% end

Normalizer = 1;

%Calculate probability
if z(k) >= 0 && z(k) <=z_max
    %P_hit = 1/ sigma_hit / sqrt(2*pi) * exp(-(z-mean)^2 / 2/ sigma_hit^2);
    P_hit =  normpdf(z(k),mean,sigma_hit) / Normalizer; 
else
    P_hit = 0;
end

%% Probability of unexpected objects
lambda_short = .05; %% Specific to map
Normalizer = 1 - exp(-lambda_short * mean);
%Calculate probability
if z(k) >= 0 && z(k) <=mean
   P_short = lambda_short* exp(-lambda_short * z(k))/Normalizer;
else
   P_short = 0; 
end

%% Probability of Failure to reach a point (PMAX)
if z(k) == z_max
    P_max = 1;
else 
     P_max= 0;
end

%% Probability of Random readings
if z(k) >= 0 && z(k) <= z_max
    P_rand = 1/z_max;
else 
     P_rand= 0;
end

%%Calculate total probability and likelihood
p = weight_hit * P_hit + weight_short * P_short + weight_max * P_max + weight_rand* P_rand;
q = q* p;

end

output(1) = q;
output(2) = mean(1);