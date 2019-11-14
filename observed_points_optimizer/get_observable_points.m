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

function observable_points = get_observable_points(asteroid_vertices, asteroid_normals, sc_position, sun_position, sc_type)
%GET_OBSERVABLE_POINTS Returns a vector of the asteroid vertex indicies
%which are feasible for observation by the spacecraft in the given position
%
%   WARNING: Sun and Spacecraft angles not being calculated correctly. The
%    surface normal vector "r_normal" uses a spherical approximation, which
%    is inaccurate. 

flag_use_instruments = true; % if false, will use simplified function, not derived from science

% Define useful function
get_angle =@(x,y) atan2(norm(cross(x(:), y(:))), dot(x(:), y(:))); % Returns angle between two vectors

% Get information
[sun_angle_ranges, sc_angle_ranges, distance_ranges, ~] = get_instrument_constraints(sc_type);
Nv = size(asteroid_vertices,1);

%% Get Observable Points
if flag_use_instruments==true
    %% Determine Points that meet instrument constraints 
    if ismember(4,sc_type) || ismember(6,sc_type) % for Altimeter or Magnetometer just retun nadir
        observable_points = get_nadir_point(asteroid_vertices, sc_position);
    else
        vertex_observability_status = zeros(1,Nv); % 1 if observable, zero otherwise
        
        for i_v = 1:Nv
            
            r_vertices = asteroid_vertices(i_v, :);
            r_normal = asteroid_normals(i_v, :);
            
            % Check altitude range
            sc_altitude = norm(sc_position(:) - r_vertices(:)); % height of spacecraft above point i_v
            if is_in_range(sc_altitude, distance_ranges) % Altitude check
                
                % Check sc angle range
                sc_angle = get_angle(r_normal, sc_position);
                if is_in_range(sc_angle, sc_angle_ranges)
                    
                    % Check sun angle range
                    sun_angle = get_angle(r_normal, sun_position);
                    if is_in_range(sun_angle, sun_angle_ranges)
                        
                        % Vertex has passed tests, it must be observable
                        vertex_observability_status(i_v) = 1;
                    end
                end
            end
        end
        
        observable_points = find(vertex_observability_status==1);
        if isempty(observable_points)
            observable_points = []; 
        end
    end
else
    %% Return All Points within n Degrees of Nadir
    rad_threshold = deg2rad(15);
    
    off_nadir_angle = zeros(1,Nv);
    for i_v = 1:Nv
        r_v = asteroid_vertices(i_v,:); % vector to asteroid vertex i_v from cg
        r_vs = sc_position - r_v; % vector from vertex i_v to spacecraft
        off_nadir_angle(i_v) = get_angle(r_v, r_vs); %  atan2(norm(cross(r_v, r_vs)), dot(r_v,r_vs));
    end
    observable_points = find(off_nadir_angle<rad_threshold);
end

end

function in_range = is_in_range(x, range_cell)
% Checks whether value is in one of the acceptable ranges of the
% range_cell (range_cell{i} contains one of the acceptable ranges)

in_range = false;
for i = 1:length(range_cell)
    if (x>=range_cell{i}(1))&&(x<=range_cell{i}(2))
        in_range=true;
    end
end

end
