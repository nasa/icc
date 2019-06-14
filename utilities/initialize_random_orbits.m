function [sc_init_pos] = initialize_random_orbits(n_spacecraft , AsteroidParameters, sc_carrier_state)
%F_INITIALIZE_RANDOM_ORBITS Initializes random orbital position around an
%asteroid 
%   Syntax: initialize_random_orbits(n_spacecraft , AsteroidParameters, sc_carrier_state)
%
%   Inputs: 
%    - num_sc: number of spacecraft to initialize orbits for
%    - AsteroidParameters
%    - sc_carrier_state [km, km/s]:  position and velocity of carrier
%      spacecraft
%   Outputs: 
%    - SC_init_pos: array containing randomly initialized states
%      (position and velocity) of the "n_spacecraft" instrument spacecraft


%% Initialize States
sc_init_pos = zeros(n_spacecraft,6);

min_radius_orbit = 2*AsteroidParameters.radius;

i_sc = 0;
while i_sc < n_spacecraft
    i_sc = i_sc + 1;
    
    if rand <= 0.2  || nargin<=2
        rand_x = 2*(rand-0.5);
        rand_y = 2*(rand-0.5);
        rand_z = 2*(rand-0.5);
        rand_point = min_radius_orbit*[rand_x rand_y rand_z]/(norm([rand_x rand_y rand_z]));
    else
        rand_x = 2*(rand-0.5);
        rand_y = 2*(rand-0.5);
        rand_z = 2*(rand-0.5);
        rand_point = (min_radius_orbit + rand*(norm(sc_carrier_state)-min_radius_orbit))*[rand_x rand_y rand_z]/(norm([rand_x rand_y rand_z]));
        
    end
    
    sc_init_pos(i_sc,1:3) = rand_point; % [km]
    
    % Given Position, find velocity
    v_magnitude = (1e-3) * sqrt(AsteroidParameters.Gravity.GM/norm(rand_point*1e3)); % [km/sec]
    
    rand_x = 2*(rand-0.5);
    rand_y = 2*(rand-0.5);
    rand_z = 2*(rand-0.5);
    
    v_direction = cross( rand_point/norm(rand_point) , [rand_x rand_y rand_z]/(norm([rand_x rand_y rand_z])) );
    v_direction = v_direction/norm(v_direction);
    
    sc_init_pos(i_sc,4:6) = v_magnitude*v_direction; % [km/sec]

    
end


end




