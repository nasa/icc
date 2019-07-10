%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

function observable_points = get_observable_points(asteroid_radius, asteroid_vertices, sc_position, sun_state, sc_obs_type)
%GET_OBSERVABLE_POINTS Returns a vector of the asteroid vertex indicies
%which are feasible for observation by the spacecraft.
%   This function defines which points on the spacecraft are acceptable for
%   observation 
%
%   Syntax:  observable_points = get_observable_points(asteroid_radius, asteroid_vertices, sc_position, sun_state, sc_obs_type)
%   
%   The current version of this function returns the nadir point as the
%   only observable point. 

%% Return Nadir Point 
Nv = size(asteroid_vertices,1);

off_nadir_angle = zeros(1,Nv);
for i_v = 1:Nv
    r_v = asteroid_vertices(i_v,:); % vector to asteroid vertex i_v from cg
    r_vs = sc_position - r_v; % vector from vertex i_v to spacecraft
    off_nadir_angle(i_v) = atan2(norm(cross(r_v, r_vs)), dot(r_v,r_vs));
end

[~, min_ind] = min(off_nadir_angle);

observable_points = min_ind;

%% Return Closest Point
% Nv = size(asteroid_vertices,1);
% 
% dist_to_sc = zeros(1,Nv);
% for i_v = 1:Nv
%     dist_to_sc(i_v) = norm(sc_position - asteroid_vertices(i_v, :));
% end
% 
% [~, min_ind] = min(dist_to_sc);
% 
% observable_points = min_ind;

end