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

function Swarm = observed_points_optimizer_main(AsteroidModel, Swarm, sc_optimized, sc_find_observable_pts)
%OBSERVED_POINTS_OPTIMIZER  Determines which points the spacecraft will
%observe given their orbits.
%   Note: The third and fourth input arguments are optional. They provide 
%   the capability for iterative use of the optmizer.  
%
%   Syntax: Swarm = observed_points_optimizer_main(AsteroidModel, Swarm, sc_optimized, sc_find_observable_pts)
%    *optional input
%   
%   Inputs: 
%    - AsteroidModel
%    - Swarm
%    - *sc_optimized: (optional) specifies the subset of agents whose
%        observation points will be optimized.
%        e.g. if sc_optimized = [1,3] then observation parameters will only
%        be calculated for agents 1 and 3
%    - *sc_find_observable_points: (optional) specifies the subset of
%        agents in sc_optmized for which the "observable_points_map" will
%        be computed. This should only be applied to agents whose observed
%        points have already been calculated though a previous call to this
%        function. 
% 
%   Outputs: 
%    - Swarm


if nargin<3
    sc_optimized=Swarm.which_trajectories_set(); % Optimization performed on these agents
end
if nargin<4
    sc_find_observable_pts=sc_optimized; % Points to calculate observable_points_map for (should be subset of sc_optimized)
end

%% Define Parameters
bits_per_point = 8*0.4*1e9; % 0.4GB, data collected at each point

flag_optimization_approach = 2; % 0 returns nadir point (no optimization); 1 for sequential optmization (shortcut); 2 for batch optmization (optimal)

%% Setup

sc_type = Swarm.Parameters.types; % 0 for carrier; 1 for instrument carrying spacecraft
K = Swarm.get_num_timesteps(); % number of time samples
N = Swarm.get_num_spacecraft(); % number of spacecraft
Nv = size(AsteroidModel.BodyModel.shape.vertices,1); % number of vertices in shape model
asteroid_vertices = AsteroidModel.BodyModel.shape.vertices; % Verticies composing surface of asteroid

%% Get Set of Feasible Observation Points at Each Timestep
observable_points = Swarm.Observation.observable_points; 
for i_time = 1:K
    for i_sc = sc_find_observable_pts
        if ismember(0,sc_type{i_sc})
            observable_points{i_sc, i_time} = []; % carrier spacecraft does not observe anything
        else
            if flag_optimization_approach==0
                observable_points{i_sc, i_time} = get_nadir_point(asteroid_vertices, Swarm.rel_trajectory_array(i_time, 1:3, i_sc ) ) ;
            else
                observable_points{i_sc, i_time} = get_observable_points(asteroid_vertices, Swarm.rel_trajectory_array(i_time, 1:3, i_sc ) ) ;
            end
        end
    end
end

% Create observable points map
if flag_optimization_approach~=0
    observable_points_map = cell(1,N);
    for i_sc = 1:N
        observable_points_map{i_sc} = sparse(zeros(Nv,K));
        for i_time = 1:K
            observable_points_map{i_sc}(observable_points{i_sc, i_time}, i_time) = 1;
        end
    end
end

Swarm.Observation.observable_points = observable_points;

%% Define Coverage Reward Map 
reward_map = cell(1,N);
if flag_optimization_approach==0
    for i_sc = sc_optimized
        reward_map{i_sc} = ones(Nv, K);
    end
else
    reward_map = get_coverage_reward_map(AsteroidModel, observable_points_map); 
end

%% Choose Observation Points
if flag_optimization_approach==0
    % Store observable points in observed points
    for i_sc = sc_optimized
        for i_time = 1:K
            Swarm.Observation.observed_points(i_sc,i_time) = observable_points{i_sc, i_time}(1);
            Swarm.Observation.priority(i_sc, i_time) = reward_map(observable_points{i_sc, i_time}(1), i_time);
        end
    end
elseif flag_optimization_approach==1 % Sequential optimization
    for i_sc = sc_optimized
        % Each agent optmizes its observation points individually, without
        % consideration of the decisions of other agents
        [Swarm.Observation.observed_points(i_sc,:), Swarm.Observation.priority(i_sc,:)] = single_agent_points_optimizer(observable_points_map{i_sc}, reward_map);
    end
else % Batch optimization
    [added_observed_points, added_priority] = swarm_points_optimizer(observable_points_map, reward_map, sc_optimized);
    for i_sc = sc_optimized
        Swarm.Observation.observed_points(i_sc,:) = added_observed_points(i_sc,:); 
        Swarm.Observation.priority(i_sc,:) = added_priority(i_sc,:); 
    end
end

%% Store Flow
Swarm.Observation.flow = bits_per_point.*sign(Swarm.Observation.observed_points) ;

end