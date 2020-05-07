classdef SpacecraftSwarm < matlab.mixin.Copyable%  < handle
    %SWARMTRAJECTORY Stores and manipulates swarm data
    
    %% Properties
    properties(SetAccess=private, GetAccess=public)
        % Trajectory arrays must be set through set methods. 
        % Parameters and sample_times cannot be altered
        
        abs_trajectory_array % [N_TIMESTEPS x 6 x N_SPACECRAFT] Array containing the trajectory of the spacecraft in absolute frame
        rel_trajectory_array % [N_TIMESTEPS x 6 x N_SPACECRAFT] Array containing the trajectory of the spacecraft in relative frame
        sample_times % [s] Sample times for the trajectory, observation, and communication flow
        Parameters % Fixed properties of spacecraft
        state_transition_matrix % [6 x 6 x N_TIMESTEPS x N_SPACECRAFT] Array containing the state transition matrix of the spacecraft
        state_transition_matrix_frame % {N_SPACECRAFT} Cell array containing the frame in which the STM was computed
        sun_state_array % [6 X N_TIMESTEPS] Array containing the trajectory of the Sun in IAU_EROS frame
    end
    
    properties(SetAccess=public, GetAccess=public)
        Observation % Variables related to observation of the small body by the spacecraft
        Communication % Variables related to communication between spacecraft
        
    end
    
    properties(SetAccess=private, GetAccess=private)
        all_trajectories_set % true | false; indicates whether trajectories have been computed
        unset_trajectories % indicies of trajectories that have not been set 
    end
    
    %% Methods
    methods
        %% Constructor Method
        function obj = SpacecraftSwarm(time_vector, sc_types, sc_max_memory)
            N = length(sc_max_memory); % Number of spacecraft in the swarm
            K = length(time_vector); % Number of time samples in horizon
            
            obj.abs_trajectory_array = zeros(K, 6, N); % Spacecraft trajectories in absolute frame
            obj.rel_trajectory_array = zeros(K, 6, N); % Spacecraft trajectories in relative frame
            obj.sample_times = time_vector; % [s]; Sample times: all time dependent variables in sync with this
            obj.state_transition_matrix = zeros(6,6,K,N); % Array containing the state transition matrix of the spacecraft
            obj.state_transition_matrix_frame = cell(N,1);
            
            obj.Parameters.available_memory = sc_max_memory; % [bits]; [1 x N] vector of the memory capacity of each spacecraft
            obj.Parameters.types = sc_types; % [1 x N] cell containing the list of instruments (indices) carried by each spacecraft. Conventionally, type 0 corresponds to the carrier spacecraft.
            obj.Parameters.n_spacecraft = N; % Number of spacecraft in the swarm
            
            obj.Observation.observed_points = zeros(N, K); % observed_points(i,k) = index of vertex on asteroid observed by spacecraft i at time k
            obj.Observation.observable_points = cell(N,K); % obvservable_points(
            obj.Observation.flow = zeros(N, K); % [bits/s]; Observation.flow(i,k) contains the data taken in by spacecraft i at time k
            obj.Observation.priority = zeros(N, K); % [reward/(bit/s)]; priority(i,k) is the value of one bit of science produced by spacecraft i at time k.
            
            obj.Communication.flow = zeros(K, N, N); % [bits/s]; Communication.flow(k,i,j) contains the data flow from s/c i to s/c j at time k
            obj.Communication.effective_source_flow = zeros(N, K); % [bits/s]; Contains the portion of obj.Observation.flow that actually reaches the carrier
            obj.Communication.bandwidths_and_memories = zeros(K, N, N);
            obj.Communication.dual_bandwidths_and_memories = zeros(K, N, N);
            
            obj.all_trajectories_set = false;
            obj.unset_trajectories = 1:N;
            
            obj.sun_state_array = get_sun_state(time_vector); % Sun trajectory in IAU_EROS frame
            
        end
        
        %% Set Methods
        
        %%%%%%%%%%%%%%%%%% Integrate all trajectories %%%%%%%%%%%%%%%%%%%%%
        function obj = integrate_trajectories(obj,ErosGravity, sc_initial_state_array, abs_or_rel)
            if nargin<4
                abs_or_rel = 'absolute'; % which frame to conduct the integration in
            end
            for i_sc = 1:size(sc_initial_state_array,1)
                sc_initial_state = sc_initial_state_array(i_sc,:);
                obj.integrate_trajectory(i_sc, ErosGravity, sc_initial_state, abs_or_rel);
            end
        end
        
        %%%%%%%%%%%%%%%%%% Integrate a single trajectory %%%%%%%%%%%%%%%%%%
        function obj = integrate_trajectory( obj, i_sc, ErosGravity, sc_initial_state, abs_or_rel)
            obj.unset_trajectories(obj.unset_trajectories==i_sc)=[]; % remove this trajectory from
            if isempty(obj.unset_trajectories())
                obj.all_trajectories_set = true;
            end

            if nargin<5
                abs_or_rel = 'absolute'; % which frame to conduct the integration in
            end
            [~, abs_traj, rel_traj, mode, stm] = ErosGravity.integrate(obj.sample_times, transpose(sc_initial_state), abs_or_rel );
            obj.abs_trajectory_array(:,:,i_sc) = abs_traj;
            obj.rel_trajectory_array(:,:,i_sc) = rel_traj;
            obj.state_transition_matrix(:,:,:,i_sc) = stm;
            obj.state_transition_matrix_frame{i_sc} = mode;
            obj.unset_trajectories(obj.unset_trajectories==i_sc)=[]; % remove this trajectory from
            if isempty(obj.unset_trajectories())
                obj.all_trajectories_set = true;
            end
        end
        
        %% Get Methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% Sizes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function n_spacecraft = get_num_spacecraft(obj)
            n_spacecraft = size(obj.abs_trajectory_array, 3);
        end
        
        function n_timesteps = get_num_timesteps(obj)
            n_timesteps = size(obj.abs_trajectory_array, 1);
        end
        
        %$%%%%%%%%%%%%%%%%%%%%%%%% Indexing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function sc_indicies = get_indicies_of_type(obj, type)
            % Returns the indicies of all spacecraft of the given type
            sc_indicies = [] ;
            for i_sc = 1:length(obj.Parameters.types)
                if ismember(type, obj.Parameters.types{i_sc})
                    sc_indicies = [sc_indicies, i_sc]; %#ok<AGROW>
                end
            end
        end
        
        function sc_indicies = get_indicies_except_type(obj, type)
            % Returns the indicies of all spacecraft not of the given type
            sc_indicies = setdiff(1:obj.get_num_spacecraft(),  obj.get_indicies_of_type(type));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%% Observation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function observed_points_list = get_observed_points_list(obj, stop_time)
            if nargin<2
                observed_points_list = unique(obj.Observation.observed_points(:));
            else
                observed_points = obj.Observation.observed_points(:, 1:stop_time);
                observed_points_list = unique(observed_points(:));
            end
        end
        
        function coverage_reward = get_coverage_reward(obj, stop_time)
            if nargin<2
                coverage_reward = sum(sum(obj.Observation.flow.*obj.Observation.priority));
            else
                coverage_reward = sum(sum(obj.Observation.flow(1:stop_time).*obj.Observation.priority(1:stop_time)));
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%% Communication %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function total_reward = get_total_reward(obj, stop_time)
            if nargin<2
                total_reward = sum(sum(obj.Communication.effective_source_flow.*obj.Observation.priority));
            else
                total_reward = sum(sum(obj.Communication.effective_source_flow(1:stop_time).*obj.Observation.priority(1:stop_time)));
            end
        end
        
        
        %% Validity and Initialization Check(s)
        function valid = is_valid(obj)
            valid = true; 
            N = obj.get_num_spacecraft(); 
            K = obj.get_num_timesteps(); 
            
            if length(fieldnames(obj))~=9
                warning('Extra fields added to object!')
                valid = false; 
            elseif length(obj.Parameters.available_memory)~=obj.get_num_spacecraft()
                warning('object.Parameters.available_memory should be [1 x N] double!')
                valid = false;
            elseif (length(obj.Parameters.types)~=obj.get_num_spacecraft()) || ~iscell(obj.Parameters.types)
                warning('object.Parameters.types should be [1 x N] cell array!')
                valid = false;
            elseif length(fieldnames(obj.Observation))~=4
                warning('Extra fields added to object.Observation!')
                valid = false;
            elseif length(fieldnames(obj.Communication))~=4
                warning('Extra fields added to object.Communication!')
                valid = false;
            elseif ~isnumeric(obj.Observation.observed_points) || (size(obj.Observation.observed_points,1)~=N) || (size(obj.Observation.observed_points,2)~=K)
                warning('object.Observation.observed_points should be [N x K] double!')
                valid = false;
            elseif ~isnumeric(obj.Observation.flow) || (size(obj.Observation.flow,1)~=N) || (size(obj.Observation.flow,2)~=K)
                warning('object.Observation.flow should be [N x K] double!')
                valid = false;
            elseif ~isnumeric(obj.Observation.priority) || (size(obj.Observation.priority,1)~=N) || (size(obj.Observation.priority,2)~=K)
                warning('object.Observation.priority should be [N x K] double!')
                valid = false;
            elseif ~iscell(obj.Observation.observable_points) || (size(obj.Observation.observable_points,1)~=N) || (size(obj.Observation.observable_points,2)~=K)
                warning('object.Observation.observable_points should be [N x K] cell!')
                valid = false;
            elseif ~isnumeric(obj.Communication.flow) || (size(obj.Communication.flow,1)~=K) || (size(obj.Communication.flow,2)~=N) || (size(obj.Communication.flow,3)~=N)
                warning('object.Communication.flow should be [K x N x N] double!')
                valid = false;
            elseif ~isnumeric(obj.Communication.bandwidths_and_memories) || (size(obj.Communication.bandwidths_and_memories,1)~=K) || (size(obj.Communication.bandwidths_and_memories,2)~=N) || (size(obj.Communication.bandwidths_and_memories,3)~=N)
                warning('object.Communication.bandwidths_and_memories should be [K x N x N] double!')
                valid = false;
            elseif ~isnumeric(obj.Communication.dual_bandwidths_and_memories) || (size(obj.Communication.dual_bandwidths_and_memories,1)~=K) || (size(obj.Communication.dual_bandwidths_and_memories,2)~=N) || (size(obj.Communication.dual_bandwidths_and_memories,3)~=N)
                warning('object.Communication.dual_bandwidths_and_memories should be [K x N x N] double!')
                valid = false;
            elseif ~isnumeric(obj.Communication.effective_source_flow) || (size(obj.Communication.effective_source_flow,1)~=N) || (size(obj.Communication.effective_source_flow,2)~=K)
                warning('object.Communication.effective_source_flow should be [N x K] double!')
                valid = false;
            end
            
        end
        
        function answer = is_trajectory_initialized(obj)
            % Indicates whether the orbits have been propagated
            answer = obj.all_trajectories_set;
        end
        
        function answer = which_trajectories_not_set(obj)
            % Indicates which orbits have not been propagated
            answer = obj.unset_trajectories;
        end

        function answer = which_trajectories_set(obj)
            % Indicates which orbits have been propagated
            answer = setdiff(1:obj.get_num_spacecraft(),obj.unset_trajectories);
        end
        
        %% Other Utilities
        function collision_occurs = collision_with_asteroid(obj, AsteroidModel)
            %COLLISION_WITH_ASTEROID Checks whether any of the trajectories
            %enter a sphere of the given radius
            %   Syntax: collision_occurs = collision_with_asteroid(obj, AsteroidModel)
            %
            %   Inputs:
            %    - AsteroidModel.BodyModel.shape.maxRadius: [km] Radius of the asteroid
            %
            %   Outputs:
            %    - collision_occurs: logical variable that is true if
            %       collision has been detected
            
            radius = AsteroidModel.BodyModel.shape.maxRadius*1000;
            collision_occurs = false;
            
            for i_timestep = 1:obj.get_num_timesteps()
                for i_sc = 1:obj.get_num_spacecraft()
                    if vecnorm(obj.abs_trajectory_array(i_timestep, 1:3, i_sc)) < radius
                        collision_occurs = true;
                        break
                    end
                end
            end
            
        end
        

    end
    
    
end

