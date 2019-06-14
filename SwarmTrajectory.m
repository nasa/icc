classdef SwarmTrajectory < handle
    %SWARMSTATEHANDLER Stores and manipulates spacecraft trajectory data

    %% Properties
    properties
        nSpacecraft % Number of spacecraft in the swarm
        trajectoryArray % [N_TIMESTEPS x 6 x N_SPACECRAFT] Array containing the trajectory of the spacecraft
        timeVector % [s] Timestamps for the trajectory
        nTimesteps  % Number of timesteps in the stored trajectory
        delta_t % [s] Temporal resolution
    end
    
    %% Methods
    methods
        %% Constructor Method
        function obj = SwarmTrajectory(swarm_traj, swarm_traj_times)
            %SWARMTRAJECTORY Construct an instance of this class
            
            % Infer swarm properties
            obj.nSpacecraft = size(swarm_traj,3);
            obj.nTimesteps = length(swarm_traj_times);
            obj.delta_t = swarm_traj_times(2)-swarm_traj_times(1);
            obj.timeVector = swarm_traj_times;
            
            % Set trajectory array
            obj.trajectoryArray = swarm_traj;
            
        end
        
        %% Get Methods
        function sc_position_array = get_position_array(obj, indicies)            
            if nargin < 2
                indicies = 1:obj.nSpacecraft;
            end
            
            sc_position_array = zeros(obj.nTimesteps, 3, length(indicies) );
            
            for i_sc = 1:length(indicies)
                sc_position_array(:, :, i_sc) = obj.trajectoryArray(:, 1:3, indicies(i_sc));
            end
        end
        
        function sc_state_array = get_state_array(obj, sc_indicies)            
            if nargin < 2
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_state_array = zeros(obj.nTimesteps, 6, length(sc_indicies) );
            
            for i_sc = 1:length(sc_indicies)
                sc_state_array(:, :, i_sc) = obj.trajectoryArray(:, 1:6, sc_indicies(i_sc));
            end
        end
        
        function sc_current_state_matrix = get_current_state( obj, i_timestep , sc_indicies )
            
            if nargin < 3
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_current_state_matrix = zeros(length(sc_indicies), 6 );
            
            for i_sc = 1:length(sc_indicies)
                sc_current_state_matrix(i_sc , :) = obj.trajectoryArray(i_timestep, : , sc_indicies(i_sc));
            end
            
        end
        
        function sc_current_position_matrix = get_current_position( obj, i_timestep , sc_indicies )
            
            if nargin < 3
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_current_position_matrix = zeros(length(sc_indicies), 3 );
            
            for i_sc = 1:length(sc_indicies)
                sc_current_position_matrix(i_sc , :) = obj.trajectoryArray(i_timestep, 1:3 , sc_indicies(i_sc));
            end
            
        end
        
        %% Other Utilities 
        function collision_occurs = collision_with_asteroid(obj, radius)
            %COLLISION_WITH_ASTEROID Checks whether any of the trajectories
            %enter a sphere of the given radius
            %   Syntax: collision_occurs = collision_with_asteroid(obj, radius)
            %   
            %   Inputs: 
            %    - radius: [m] Radius of the asteroid 
            % 
            %   Outputs: 
            %    - collision_occurs: logical variable that is true if
            %       collision has been detected 

            
            collision_occurs = false; 
            
            for i_timestep = 1:length(obj.timeVector)
                for i_sc = 1:obj.nSpacecraft
                    if vecnorm(obj.trajectoryArray(i_timestep, 1:3, i_sc)) < radius
                        collision_occurs = true;
                        break
                    end
                end
            end
            
        end

    end
    
    
end 
    
