classdef SwarmTrajectory < handle
    %SWARMSTATEHANDLER Stores and manipulates spacecraft trajectory data
    %   The state of a spacecraft is a vector containing its position and
    %   velocity x = [s_x s_y s_z v_x v_y v_z]
    %   - positions are in [m], velocity in [m/s]
    
    %% Properties
    properties
        nSpacecraft % number of spacecraft in the swarm
        trajectoryArray % trajectoryArray(i_time, i_state, i_spacecraft) contains state information for the spacecraft at a given timestep
        timeVector % [s] timestamps for the trajectory 
        nTimesteps  % number of timesteps in the stored trajectory
        delta_t % [s] temporal resolution 
    end
    
    %% Methods
    methods
        %% Constructor Method
        function obj = SwarmTrajectory(swarm_traj, swarm_traj_times)
            %SWARMTRAJECTORY Construct an instance of this class
            %   Reorganizes trajectory information into neat array with
            %   convienient methods for retrieving the data.
            %   #DO: add option to specify delta_t
            
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
            %METHOD1 .
            %   ...
            
            if nargin < 2
                indicies = 1:obj.nSpacecraft;
            end
            
            sc_position_array = zeros(obj.nTimesteps, 3, length(indicies) );
            
            for i_sc = 1:length(indicies)
                sc_position_array(:, :, i_sc) = obj.trajectoryArray(:, 1:3, indicies(i_sc));
            end
        end
        
        function sc_state_array = get_state_array(obj, sc_indicies)
            %METHOD1 .
            %   ...
            
            if nargin < 2
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_state_array = zeros(obj.nTimesteps, 6, length(sc_indicies) );
            
            for i_sc = 1:length(sc_indicies)
                sc_state_array(:, :, i_sc) = obj.trajectoryArray(:, 1:6, sc_indicies(i_sc));
            end
        end
        
        function sc_current_state_matrix = get_current_state( obj, i_timestep , sc_indicies )
            %TBD
            %   TBD
            
            if nargin < 3
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_current_state_matrix = zeros(length(sc_indicies), 6 );
            
            for i_sc = 1:length(sc_indicies)
                sc_current_state_matrix(i_sc , :) = obj.trajectoryArray(i_timestep, : , sc_indicies(i_sc));
            end
            
        end
        
        function sc_current_position_matrix = get_current_position( obj, i_timestep , sc_indicies )
            %TBD
            %   TBD
            
            if nargin < 3
                sc_indicies = 1:obj.nSpacecraft;
            end
            
            sc_current_position_matrix = zeros(length(sc_indicies), 3 );
            
            for i_sc = 1:length(sc_indicies)
                sc_current_position_matrix(i_sc , :) = obj.trajectoryArray(i_timestep, 1:3 , sc_indicies(i_sc));
            end
            
        end
        
    end
end















