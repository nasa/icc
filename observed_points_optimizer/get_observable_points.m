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

function observable_points = get_observable_points(AsteroidModel, Swarm, i_time, i_sc)
%GET_OBSERVABLE_POINTS Returns a vector of the asteroid vertex indicies
%which are feasible for observation by the spacecraft in the given position

asteroid_vertices = AsteroidModel.BodyModel.shape.faceCenters; % faceCenters composing surface of asteroid
asteroid_normals = AsteroidModel.BodyModel.shape.faceNormals; % Normals at faceCenters
sc_position = Swarm.rel_trajectory_array(i_time, 1:3, i_sc );
sun_position = Swarm.sun_state_array(1:3,i_time)';
sc_type = Swarm.Parameters.types{i_sc};

next_sc_position = sc_position;
if i_time < Swarm.get_num_timesteps()
    next_sc_position = Swarm.rel_trajectory_array(i_time+1, 1:3, i_sc );
end

flag_use_instruments = true; % if false, will use simplified function, not derived from science

% Define useful function
% get_angle =@(x,y) atan2(norm(cross(x(:), y(:))), dot(x(:), y(:))); % Returns angle between two vectors using tan
get_angle =@(x,y) acos( dot(x(:), y(:)) / (norm(x(:)) * norm(y(:))) ); % Returns angle between two vectors using cos

% Get information
[sun_angle_ranges, sc_angle_ranges, distance_ranges] = get_instrument_constraints(sc_type);
Nv = size(asteroid_vertices,1);

%% Get Observable Points
if flag_use_instruments==true
    %% Determine Points that meet instrument constraints 
    if ismember(0,sc_type) % For carrier, nothing is observed
        observable_points = [];
%     elseif ismember(4,sc_type) || ismember(6,sc_type) % for Altimeter or Magnetometer just retun nadir
%         observable_points = get_nadir_point(AsteroidModel, Swarm, i_time, i_sc);
    else
        vertex_observability_status = zeros(1,Nv); % 1 if observable, zero otherwise
        
        for i_v = 1:Nv
            
            r_vertices = asteroid_vertices(i_v, :);
            r_normal = asteroid_normals(i_v, :);
            
            % Check altitude range
            sc_altitude = norm(sc_position - r_vertices); % height of spacecraft above point i_v
            next_sc_altitude = norm(next_sc_position - r_vertices); % height of spacecraft above point i_v
            if (is_in_range_dist(sc_altitude, distance_ranges)) && (is_in_range_dist(next_sc_altitude, distance_ranges)) % Altitude check
                
                % Check sc angle range
                sc_angle = get_angle(r_normal, sc_position-r_vertices);
                next_sc_angle = get_angle(r_normal, next_sc_position-r_vertices);
                if (is_in_range_angle(sc_angle, sc_angle_ranges)) && (is_in_range_angle(next_sc_angle, sc_angle_ranges))
                    
                    % Check sun angle range
                    sun_angle = get_angle(r_normal, sun_position-r_vertices);
                    if is_in_range_angle(sun_angle, sun_angle_ranges)
%                         disp("SC in sun angle range, vertex OK")
                        % Vertex has passed tests, it must be observable
                        vertex_observability_status(i_v) = 1;
%                     else
%                         fprintf("SC not in sun angle range (sun angle %f, bounds (%f to %f))\n",sun_angle, sun_angle_ranges{1}(1), sun_angle_ranges{1}(2))
                    end
%                 else
%                     fprintf("SC not in view angle range (view angle %f, bounds (%f to %f))\n",sc_angle, sc_angle_ranges{1}(1), sc_angle_ranges{1}(2))
                end
%             else
%                 fprintf("SC not in altitude range (altitude %f, bounds (%f-%f))\n",sc_altitude, distance_ranges{1}(1), distance_ranges{1}(2))
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

function in_range = is_in_range_dist(x, range_cell)
% Checks whether value is in one of the acceptable ranges of the
% range_cell (range_cell{i} contains one of the acceptable ranges)

in_range = false;
for i = 1:length(range_cell)
    if (x>=range_cell{i}(1))&&(x<=range_cell{i}(2))
        in_range=true;
    end
end

end


function in_range = is_in_range_angle(x, range_cell)

in_range = false;
for i = 1:length(range_cell)
    if (x<= max(range_cell{i}(1), range_cell{i}(2))) && (x>= min(range_cell{i}(1), range_cell{i}(2)))
        in_range=true;
    end
end
end