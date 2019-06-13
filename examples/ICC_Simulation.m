%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                    Usage example of ICC simulation                      %
%                                                                         %
% Demonstrates the usage of various utility functions in the context of   %
% creating a complete icc simulation. Heuristics are used to determine    %
% design parameters such as communication flow and initial state of the   %
% spacecraft.                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(strcat(ROOT_PATH,'/utilities'))
addpath(strcat(ROOT_PATH,'/visualization'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 10; % Number of Spacecraft

bits_per_point = 8*0.4*1e9; % 0.4GB, data collected at each point
standard_comm_data_rate = 50e3; % 50kbps at 100km, SC to carrier datarate
sc_max_memory = 8*0.5e12; % 0.5TB, on-board memory on each SC

delta_t = 10*60; % [sec], simulation time step
total_t = 0.5*24*60*60; % [sec]; 0.5 days, total time of simulation

% Set Simulation Flags:
flag_show_plots = 1; % Put this to 1 to show plots, 0 to not show plots, put 2 to show plot only in end
flag_store_video = 0; % Put this to 1 to store videos, only works if flag_show_plots = 1

flag_SC_observing_points = 1; % Put this to 0 for SC to observe all points per unit time, 1 for observing only nearest point per unit time, 2 for observing only nearest point per obs_time_threshold

% Set Plotting Preferences
standard_font_size = 18; % size of font in the simulation figure
color_array = ['c' 'r' 'b' 'g' 'm']; % colors used to index spacecraft

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initialize Simulation Environment Variables                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};

% Load physical parameters of asteroid as a struct
ErosParameters = get_Eros_body_parameters(SphericalModel{1}); 
ErosParameters.radius = ErosParameters.radius*10^-3; % change units from [m] to [km]  % #FIX 

% Name and path of shapefile. For now, only used for plotting.
shapefilename = 'EROS 433/MSI_optical_plate_models/eros022540.tab';

ErosShapeModel = get_shape_model(shapefilename) ; % Faces and verticies of Eros model
pos_points = ErosShapeModel.Vertices; 
num_points = size(pos_points,1);

% Create an instance of the model we will use
ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,ErosParameters.rotation_rate,0,ErosShapeModel,@ode23tb);

% Initialize point index. This keeps track of the observed points on Eros. 
point_index = zeros(num_points,1);

% Initialize memory use array. This keeps track of the memory usage for sc
sc_memory_use = zeros(n_spacecraft,1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Integrate Trajectory                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize carrier state
carrier_current_state = [100 0 0]; % [km]
v_magnitude = (1e-3) * sqrt(ErosParameters.Gravity.GM/norm(carrier_current_state*1e3)); % [km/sec]
v_direction = [0 1 0];
carrier_sc_velocity = v_magnitude*v_direction; % [km/sec]
carrier_current_state = [carrier_current_state carrier_sc_velocity];

% Initialize other spacecraft states (i.e. swarm state)
sc_initial_state_array = initialize_random_orbits(n_spacecraft, ErosParameters); % #FIX: third input arg (optional) messes something up down the line

% Initialize spacecraft state array
sc_current_state = sc_initial_state_array;

timeVector = 0:delta_t:total_t; 

% Integrate for instrument spacecraft 
sc_state_array = zeros( length(timeVector), 6, n_spacecraft);
for i_sc = 1:n_spacecraft
    [~, abs_traj, ~] = ErosGravity.integrate(timeVector, sc_initial_state_array(i_sc, :).*10^3, 'absolute' );
    sc_state_array(:, :, i_sc) = abs_traj;
end

% Integrate for carrier spacecraft 
[~ ,abs_traj, ~] = ErosGravity.integrate(timeVector, carrier_current_state.*10^3, 'absolute' ); % [m]
carrier_state_array = abs_traj; 

% Instantiate SwarmTrajectory class for handling orbit data
swarm   = SwarmTrajectory(sc_state_array, timeVector); % instrument and relay spacecraft
carrier = SwarmTrajectory(carrier_state_array, timeVector); % carrier spacecraft

    % Collision Check
    if min(vecnorm(sc_current_state(:,1:3),2,2)) < ErosParameters.radius
        disp('Crashed into Asteroid!');
        percentage_seen = 0;
       
        return
        
    end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Simulation                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i_timestep = 0;
sc_position_array = swarm.get_position_array().*10^-3; % [km] array is indexed as: (i_timestep, i_xyz, i_spacecraft)
carrier_position_array = carrier.get_position_array().*10^-3; % [km] array is indexed as: (i_timestep, i_xyz, i_spacecraft)
for t=timeVector % time loop
        
    i_timestep = i_timestep+1; % indexes "current" point in discrete time trajectory
    
    % Load states at current timestep
    carrier_current_state = carrier.get_current_position(i_timestep).*10^-3; % [km]
    sc_previous_state = sc_current_state;
    sc_current_state = swarm.get_current_state(i_timestep).*10^-3; % [km]
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                     Communication with Carrier                      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulate single-hop relay of information from instument spacecraft to
    % carrier, using a heuristic to decide which spacecraft sends its data
    [communicating_sc_index, sc_memory_use] = sc_communication_single_hop(sc_previous_state, sc_current_state, ...
        carrier_current_state, ErosParameters, standard_comm_data_rate, sc_memory_use, sc_max_memory, delta_t); 
    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                      Observation of Asteroid                        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update location of points on asteroid to account for its rotation
    R = rotation_matrix_at_t(t,ErosParameters.rotation_rate);
    ErosShapeModel.Vertices = (R*pos_points')';
    
    % Simulate observation of the asteroid by the instrument spacecraft
    % (i.e. collection of data)
    [point_index, sc_memory_use] = sc_observing_points(sc_memory_use, sc_max_memory, sc_current_state,...
        point_index, ErosShapeModel.Vertices, ErosParameters, flag_SC_observing_points, bits_per_point);
    
    not_observed_index = logical(point_index == 0);
    percentage_seen = 100*(num_points - sum(not_observed_index))/num_points;
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           Visualization                             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if flag_show_plots == 1
        %% Setup the Plot 
        if i_timestep == 1 
            h1=figure(1);
            hold on 
            set(h1,'Color',[1 1 1]);
            set(h1,'units','normalized','outerposition',[0 0 1 1])
            set(h1,'PaperPositionMode','auto');
        end 
        
        %% Plot Observing Points Around Asteroid in 3D
        subplot(2,4,[1 2 5 6]);
        
        if i_timestep == 1
            initialize_spatial_plot_3d(standard_font_size);
            h_eros_patch = render_asteroid_3d(ErosShapeModel); % Make asteroid patch from shape model
        else
            delete(h_obs_pts); delete(h_sc_pos); delete(h_carrier_pos) % delete points from previous iterations 
        end
        
        set(h_eros_patch, 'Vertices', ErosShapeModel.Vertices) % Update Eros patch
        h_obs_pts = render_observed_points_3d(ErosShapeModel.Vertices, point_index, n_spacecraft, color_array, false); % Show which points have been observed by the spacecraft
        h_sc_pos = render_spacecraft_3d(sc_position_array(1:i_timestep, :, :), true, color_array);  % Plot spacecraft positions and orbital trajectories
        h_carrier_pos = render_spacecraft_3d(carrier_position_array(1:i_timestep, :, :), true, 'k'); % Plot spacecraft positions and orbital trajectories
        
        title(['Time = ',num2str(floor(t/3600)),' hours, ',num2str(percentage_seen),' % covered'],'fontsize',standard_font_size,'FontName','Times New Roman')
        axis equal; 
        view(3); 
        
        %% Plot Observing Points Around Asteroid in 2D
        subplot(2,4,3)
        cla()
        render_observed_points_2d(pos_points, point_index, n_spacecraft, 'above', color_array) % Show which points have been observed above equator
        title('Above Equator','fontsize',standard_font_size,'FontName','Times New Roman')
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        
        subplot(2,4,4)
        cla()
        render_observed_points_2d(pos_points, point_index, n_spacecraft, 'below', color_array) % Show which points have been observed below equator
        title('Below Equator','fontsize',standard_font_size,'FontName','Times New Roman')
        set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
        
        %% Plot Memory Use and Communication Topology 
        subplot(2,4,7)
        plot_memory_comparison(sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, sc_max_memory , color_array, standard_font_size )
        
        subplot(2,4,8)
        plot_communication_topology(sc_current_state, carrier_current_state, communicating_sc_index, ErosParameters.radius, color_array, standard_font_size)
        
        drawnow limitrate
    end
    
    if (percentage_seen == 100) && (max(sc_memory_use) == 0)
        break
    end
end
