%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                    Usage example of ICC simulation                      %
%                                                                         %
% Demonstrates the usage of various utility functions in the context of   %
% creating a complete icc simulation. Heuristics are used to determine    %
% design parameters such as communication flow and initial state of the   %
% spacecraft.                                                             %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities 
addpath(strcat(ROOT_PATH,'/visualization'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 2; % Number of Spacecraft, not counting the carrier

bits_per_point = 8*0.4*1e9; % 0.4GB, data collected at each point
standard_comm_data_rate = 50e3; % 50kbps at 100km, SC to carrier datarate
sc_max_memory = 8*0.5e12; % [bits] 0.5TB, on-board memory on each SC 

delta_t = 10*60; % [sec], simulation time step
total_t = 0.5*24*60*60; % [sec]; 0.5 days, total time of simulation

flag_scObservingPoints = 1; % Put this to 0 for SC to observe all points per unit time, 1 for observing only nearest point per unit time 

% Set Plotting Preferences
standard_font_size = 18; % size of font in the simulation figure
color_array = ['c' 'r' 'b' 'g' 'm']; % colors used to index spacecraft

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Initialize Simulation Environment Variables                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};

% Load physical parameters of asteroid as a struct
ErosParameters = get_Eros_body_parameters(SphericalModel{1}); % input is optional
ErosParameters.radius = ErosParameters.radius*10^-3; % change units from [m] to [km] 

% Name and path of shapefile. For now, only used for plotting.
shapefilename = 'EROS 433/MSI_optical_plate_models/eros022540.tab';

ErosShapeModel = get_shape_model(shapefilename) ; % Faces and verticies of Eros model
pos_points = ErosShapeModel.Vertices;
num_points = size(pos_points,1);

% Create an instance of the dynamics model we will use
ErosGravity = SphericalHarmonicsGravityIntegrator(SphericalModel,ErosParameters.rotation_rate, 0, ErosShapeModel, @ode23tb);

% Initialize point index. This keeps track of the observed points on Eros.
point_index = zeros(num_points,1);

% Initialize memory use array. This keeps track of the memory usage for the spacecraft
sc_memory_use = zeros(n_spacecraft,1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Integrate Trajectory                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize carrier state
carrier_current_pos = [100 0 0]; % [km]
v_magnitude = (1e-3) * sqrt(ErosParameters.Gravity.GM/norm(carrier_current_pos*1e3)); % [km/sec]
v_direction = [0 1 0];
carrier_sc_velocity = v_magnitude*v_direction; % [km/sec]
carrier_current_state = [carrier_current_pos carrier_sc_velocity];

% Initialize other spacecraft states (i.e. swarm state)
sc_initial_state_array = initialize_random_orbits(n_spacecraft, ErosParameters); % #FIX: third input arg (optional) messes something up down the line

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

% Check for collisions
if swarm.collision_with_asteroid(ErosParameters.radius*10^3)
    fprintf('\nCollision with the asteroid!\n\n')
    percentage_seen = 0;
    return
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Simulation                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i_timestep = 0;
sc_pos_array = swarm.get_position_array().*10^-3; % [km] array is indexed as: (i_timestep, i_xyz, i_spacecraft)
carrier_pos_array = carrier.get_position_array().*10^-3; % [km] array is indexed as: (i_timestep, i_xyz, i_spacecraft)
sc_current_pos = swarm.get_current_position(1).*10^-3; % [km]

for t=timeVector % time loop
    % Current time 
    i_timestep = i_timestep+1; % indexes "current" point in discrete time trajectory
    
    % Load states at current timestep
    carrier_current_pos = carrier.get_current_position(i_timestep).*10^-3; % [km]
    sc_previous_pos =  sc_current_pos; % [km]
    sc_current_pos = swarm.get_current_position(i_timestep).*10^-3; % [km]
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                     Communication with Carrier                      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Simulate single-hop relay of information from instument spacecraft to
    % carrier, using a heuristic to decide which spacecraft sends its data
    [communicating_sc_index, sc_memory_use] = simulate_single_hop_communication(sc_previous_pos, sc_current_pos, ...
        carrier_current_pos, ErosParameters.radius, standard_comm_data_rate, sc_memory_use, sc_max_memory, delta_t);
    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                      Observation of Asteroid                        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Update location of points on asteroid to account for its rotation
    R = rotation_matrix_at_t(t,ErosParameters.rotation_rate);
    ErosShapeModel.Vertices = (R*pos_points')';
    
    % Simulate observation of the asteroid by the spacecraft (data collection)
    [point_index, sc_memory_use] = simulate_asteroid_observation( sc_current_pos, ErosParameters.radius, ...
        bits_per_point, sc_memory_use, sc_max_memory, point_index, pos_points, flag_scObservingPoints );
    
    not_observed_index = logical(point_index == 0);
    percentage_seen = 100*(num_points - sum(not_observed_index))/num_points;
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           Visualization                             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Setup the Plot
    if i_timestep == 1
        h1=figure(1);
        set(h1,'Color',[1 1 1]);
        set(h1,'units','normalized','outerposition',[0 0 1 1])
        set(h1,'PaperPositionMode','auto');
    end

    %% Plot Observing Points Around Asteroid in 3D
    subplot(2,4,[1 2 5 6]);
    
    if i_timestep == 1
        axes_limits = [-1,1].*max(vecnorm(carrier_pos_array,2,2)).*1.01;
        initialize_spatial_plot_3d('fontSize',standard_font_size, 'limits', axes_limits);
        h_eros_patch = render_asteroid_3d(ErosShapeModel); % Make asteroid patch from shape model
    else
        delete(h_obs_pts); delete(h_sc_pos); delete(h_carrier_pos) % delete points from previous iterations
    end

    set(h_eros_patch, 'Vertices', ErosShapeModel.Vertices) % Update Eros patch
    h_obs_pts = render_observed_points_3d(ErosShapeModel.Vertices, point_index, n_spacecraft, ...
        'color_array', color_array, 'show_not_observed', false); % Show which points have been observed by the spacecraft
    h_sc_pos = render_spacecraft_3d(sc_pos_array(1:i_timestep, :, :),...
        'color_array', color_array, 'show_trail', true);  % Plot spacecraft positions and orbital trajectories
    h_carrier_pos = render_spacecraft_3d(carrier_pos_array(1:i_timestep, :, :),...
        'color', 'k', 'show_trail', true); % Plot spacecraft positions and orbital trajectories
    
    title(['Time = ',num2str(floor(t/3600)),' hours, ',num2str(percentage_seen),' % covered'],'fontsize',standard_font_size,'FontName','Times New Roman')
    
    %% Plot Observing Points Around Asteroid in 2D
    subplot(2,4,3)
    cla()
    render_observed_points_2d(pos_points, point_index, n_spacecraft, 'above', 'color_array', color_array) % Show which points have been observed above equator
    title('Above Equator','fontsize',standard_font_size,'FontName','Times New Roman')
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    
    subplot(2,4,4)
    cla()
    render_observed_points_2d(pos_points, point_index, n_spacecraft, 'below', 'color_array', color_array) % Show which points have been observed below equator
    title('Below Equator','fontsize',standard_font_size,'FontName','Times New Roman')
    set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
    
    %% Plot Memory Use and Communication Topology
    subplot(2,4,7)
    plot_memory_comparison(sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, sc_max_memory, color_array, standard_font_size )
    
    subplot(2,4,8)
    plot_communication_topology(sc_current_pos, carrier_current_pos, communicating_sc_index, ErosParameters.radius, color_array, standard_font_size)
    
    drawnow limitrate
    
    if (percentage_seen == 100) && (max(sc_memory_use) == 0)
        break
    end
end
