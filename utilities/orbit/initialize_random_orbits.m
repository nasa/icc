%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sc_init_pos] = initialize_random_orbits(n_spacecraft , AsteroidParameters, sc_carrier_state)
%F_INITIALIZE_RANDOM_ORBITS Initializes random orbital position around an
%asteroid 
%   Syntax: initialize_random_orbits(n_spacecraft , AsteroidParameters, sc_carrier_state)
%
%   Inputs: 
%    - num_sc: number of spacecraft to initialize orbits for
%    - AsteroidParameters
%    - sc_carrier_state [m, m/s]:  position and velocity of carrier
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
    
    sc_init_pos(i_sc,1:3) = rand_point; % [m]
    
    % Given Position, find velocity
    v_magnitude =  sqrt(AsteroidParameters.Gravity.GM/norm(rand_point)); % [m/s]
    rand_x = 2*(rand-0.5);
    rand_y = 2*(rand-0.5);
    rand_z = 2*(rand-0.5);
    
    v_direction = cross( rand_point/norm(rand_point) , [rand_x rand_y rand_z]/(norm([rand_x rand_y rand_z])) );
    v_direction = v_direction/norm(v_direction);
    
    sc_init_pos(i_sc,4:6) = v_magnitude*v_direction; % [m/s]

    
end


end




