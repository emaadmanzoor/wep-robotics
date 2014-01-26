function [ q ] = beam_range_finder_model( Z_t, x_t, y_t, sigma, map, range, W )
    % INPUT:
    %   Z_t: actual sensor reading for which you're finding probability
    %   Has a reading for different angles from 0 - 360 degrees.
    %   x_t, y_t: position on map at time t
    %   sigma: variance of the sensor
    %   map: ground-truth map, BW image
    %   range: sensor range
    %   W = [zhit, zshort, zmax, zrand] = weights for different errors
    % CALCULATE
    %   z_t_k_star: measurement from ray-casting on the map (ground-truth)
    %   q: product of probabilities of all sensor readings (for different
    %   degrees)
    q = 1;
    for theta = 0:90:360
        z_t_k_star = castraysingle(x_t, y_t, theta, map, range);
        P_hit = 0;
        if Z_t(theta) > 0 && Z_t(theta) <= range;
            
        end
        q = q * (W' * P);
    end
end