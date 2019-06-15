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
function [point_index, sc_memory_use] = simulate_asteroid_observation( sc_current_pos, asteroid_radius, bits_per_point, sc_memory_use, sc_max_memory, point_index, pos_points, flag_scObservingPoints )
%SIMULATE_ASTEROID_OBSERVATION Simulates observation of the asteroid by the
%spacecraft. Returns updated memory use vector and the indicies of the
%observed points.
%   Syntax: [point_index, sc_memory_use] = simulate_asteroid_observation(sc_memory_use, sc_max_memory,
%             sc_current_pos, point_index, pos_points, asteroid_radius,  bits_per_point, *flag_scObservingPoints )
%    *optional input
%   
%   Inputs: 
%    - sc_current_pos [m] : [N_SPACECRAFT x 3] Array of sc positions 
%    - sc_memory_use [bits]: Vector containing the current memory usage of
%    	each spacecraft 
%    - sc_max_memory [bits]: Scalar indicating the maximum memory capacity
%    	of the spacecraft 
%    - point_index: Vector of indicies for observed points on the asteroid
%    - pos_points: [N_VERTICIES x 3] Array of model vertex points 
%    - asteroid_radius: [m]
%    - bits_per_point [bits]: Data collected at a point on the body
%    - *flag_scObservingPoints: Scalar selecting observation method to use 
%        Cases: (1) Observe all points per unit time 
%               (2) Observe only nearest point per unit time 
% 
%   Outputs: 
%    - point_index: Updated vector of observed indicies 
%    - sc_memory_use [bits]: Updated vector of spacecraft memory usage


if nargin<8
    flag_scObservingPoints = 1;
end

% Infer Sizes
n_spacecraft = size(sc_memory_use,1);

% SC observing Points
for ns = 1:1:n_spacecraft
    this_SC_pos = sc_current_pos(ns,1:3);
    not_observed_index = logical(point_index == 0);
    max_observation_distance = min(2*asteroid_radius, norm(this_SC_pos));
    distance_index = logical(vecnorm(pos_points - this_SC_pos,2,2) < max_observation_distance);
    this_SC_observes = (not_observed_index & distance_index);
    if sum(this_SC_observes) > 0
        switch flag_scObservingPoints
            case 0
                if sc_memory_use(ns,1) + sum(this_SC_observes)*bits_per_point < sc_max_memory
                    point_index(this_SC_observes) = ns;
                    sc_memory_use(ns,1) = sc_memory_use(ns,1) + sum(this_SC_observes)*bits_per_point;
                elseif sc_memory_use(ns,1) + bits_per_point > sc_max_memory
                    % Do nothing
                else
                    this_indices = find(this_SC_observes)';
                    for this_index_i = this_indices
                        if sc_memory_use(ns,1) + bits_per_point < sc_max_memory
                            point_index(this_index_i) = ns;
                            sc_memory_use(ns,1) = sc_memory_use(ns,1) + bits_per_point;
                        end
                    end
                end
            case 1
                % observe only 1 point per time step
                if sc_memory_use(ns,1) + bits_per_point <= sc_max_memory
                    this_distance_vector = [vecnorm(pos_points(this_SC_observes,:) - this_SC_pos,2,2),  find(this_SC_observes)];
                    [~,I_tdv] = sort(this_distance_vector(:,1));
                    this_index_i = round(this_distance_vector(I_tdv(1),2));
                    point_index(this_index_i) = ns;
                    sc_memory_use(ns,1) = sc_memory_use(ns,1) + bits_per_point;
                end
        end
    end
end