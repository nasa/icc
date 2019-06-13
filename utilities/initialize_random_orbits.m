function [SC_init_pos] = initialize_random_orbits(num_SC , AsteroidParameters, sc_carrier_state)
%F_INITIALIZE_RANDOM_ORBIT_STATE Initializes random orbital position around
%an asteroid 
%   Inputs: - num_sc: number of spacecraft to initialize orbits for
%           - AsteroidParameters
%           - sc_carrier_state (optional): position and velocity [km, km/s]
%           of carrier spacecraft
%   Outputs: - SC_init_pos: array containing randomly initialized states
%   (position and velocity) of the "num_SC" instrument spacecraft


%% %%%%%%%%%%%%%%%%%%%%%%%%%% TO DO: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
- make seperate function that works with MC 
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize States
SC_init_pos = zeros(num_SC,6);

min_radius_orbit = 2*AsteroidParameters.radius;

ns = 0;
while ns < num_SC
    ns = ns + 1;
    
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
    
    SC_init_pos(ns,1:3) = rand_point; % [km]
    
    % Given Position, find velocity
    v_magnitude = (1e-3) * sqrt(AsteroidParameters.Gravity.GM/norm(rand_point*1e3)); % [km/sec]
    
    rand_x = 2*(rand-0.5);
    rand_y = 2*(rand-0.5);
    rand_z = 2*(rand-0.5);
    
    v_direction = cross( rand_point/norm(rand_point) , [rand_x rand_y rand_z]/(norm([rand_x rand_y rand_z])) );
    v_direction = v_direction/norm(v_direction);
    
    SC_init_pos(ns,4:6) = v_magnitude*v_direction; % [km/sec]
    
%     % Check if orbit is valid!
%     if flag_SC_motion == 4
%         disp(['SC ',num2str(ns),' trial orbit'])
%         
%         this_SC_pos_vel = SC_init_pos(ns,:);
%         
%         [t_ode_SC_full,y_ode_SC_full] = ode45(@(t_ode,y) func_real_Eros_gravity_v2(t_ode,y,rotation_rate,Eros_gravity_data), [0:delta_t:total_t+delta_t], this_SC_pos_vel);
%         
%         vec_norm_y_ode = vecnorm(y_ode_SC_full(:,1:3),2,2);
%         if ( max(vec_norm_y_ode) > 1.5*norm(carrier_SC_pos)) || ( min(vec_norm_y_ode) < Eros_Radius)
%             disp('SC Crashed!')
%             ns = ns - 1;
%         end
%         
%     end
    
end


end




