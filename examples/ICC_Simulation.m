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

rng default % Pseudo-random but repeatable scenario

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))
addpath(strcat(ROOT_PATH,'/relay_orbit_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_spacecraft = 4;  % Number of Spacecraft, not counting the carrier

sc_types = cell(1,n_spacecraft);
for i_sc = 1:n_spacecraft
    sc_types{i_sc}  = randi([1,6]); % Indicies for instruments on board
end
sc_types{n_spacecraft} = 0; % Mark the carrier so it will not be used
                            % in the Monte Carlo optimization

sc_max_memory = zeros(1,n_spacecraft); % not used, but must be defined

delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

% Number of orbits to consider in the Monte Carlo science orbit optimizer
n_trial_orbits = 10 ;

% Maximum time for the trust region relay orbit optimizer, in seconds
max_relay_optimization_time = 300;
% Which spacecraft should be optimized as relays?
relay_orbit_indices = [3, 4];
assert(max(relay_orbit_indices)<=n_spacecraft, "ERROR: relay orbit indices exceed number of spacecraft");

sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft-1); % 20 GB max memory for instrument spacecraft
sc_max_memory(1,n_spacecraft) = 8*10000*1e9; % Memory limit for carrier spacecraft

% Parameters for bandwidth model
bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Initialize Eros Model                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
addpath(strcat(SBDT_PATH,'/Startup'));
global constants
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

% In order to select a different gravity model, change the inputs to
% the loadEros function. See the help for assistance
eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
ErosModel = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Initialize Swarm Model                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Instantiate SpacecraftSwarm class for handling spacecraft data
Swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Add the carrier orbit.
carrier_index = Swarm.get_indicies_of_type(0);
if length(carrier_index)>1
    error("More than one carrier - while this may work, it is not supported")
end
carrier_initial_conditions = initialize_carrier_orbit(ErosModel);
Swarm.integrate_trajectory(carrier_index, ErosModel, carrier_initial_conditions);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                    Sciencecraft Orbit Optimization                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimize Orbits and Observed Points with Monte Carlo
Swarm = monte_carlo_coverage_optimizer_main(ErosModel, Swarm, n_trial_orbits);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         Show Coverage Results                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Relay Orbit Optimization                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

[Swarm] = relay_optimization(Swarm, ErosModel, bandwidth_parameters, relay_orbit_indices, max_relay_optimization_time);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Show Comms Results                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot_coverage(Swarm, ErosModel);
plot_communications(Swarm, ErosModel);

%% Plot Observing Points Around Asteroid in 3D

%% Plot Memory Use and Communication Topology

figure()

for time_step = 1:length(Swarm.sample_times)
    subplot(2,4,3)
    render_observed_points_2d(ErosModel, Swarm, 'above', 'time_limits', [1, time_step]) % Show which points have been observed above equator
    subplot(2,4,4)
    render_observed_points_2d(ErosModel, Swarm, 'below', 'time_limits', [1, time_step]) % Show which points have been observed above equator
    subplot(2,4,7)
    plot_memory_comparison(time_step, Swarm)
    subplot(2,4,8)
    plot_communication_topology_2d(time_step, Swarm, ErosModel)
    drawnow limitrate
end

%     
%     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %                           Visualization                             %
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%     
%     %% Setup the Plot
%     if i_timestep == 1
%         h1=figure(1);
%         set(h1,'Color',[1 1 1]);
%         set(h1,'units','normalized','outerposition',[0 0 1 1])
%         set(h1,'PaperPositionMode','auto');
%     end
% 
%     %% Plot Observing Points Around Asteroid in 3D
%     subplot(2,4,[1 2 5 6]);
%     
%     if i_timestep == 1
%         axes_limits = [-1,1].*max(vecnorm(carrier_pos_array,2,2)).*1.01; % Set axes limits to outside carrier orbit radius 
%         initialize_spatial_plot_3d('fontSize',standard_font_size, 'limits', axes_limits.*10^-3);
%         h_eros_patch = render_asteroid_3d(ErosShapeModel); % Make asteroid patch from shape model 
%     else
%         delete(h_obs_pts); delete(h_sc_pos); delete(h_carrier_pos) % delete points from previous iterations
%     end
% 
%     set(h_eros_patch, 'Vertices', ErosShapeModel.Vertices.*10^-3) % Update Eros patch
%     h_obs_pts = render_observed_points_3d(ErosShapeModel.Vertices.*10^-3, point_index, n_spacecraft, ...
%         'color_array', color_array, 'show_not_observed', false); % Show which points have been observed by the spacecraft
%     h_sc_pos = render_spacecraft_3d(sc_pos_array(1:i_timestep, :, :).*10^-3,...
%         'color_array', color_array, 'show_trail', true);  % Plot spacecraft positions and orbital trajectories
%     h_carrier_pos = render_spacecraft_3d(carrier_pos_array(1:i_timestep, :, :).*10^-3,...
%         'color', 'k', 'show_trail', true); % Plot spacecraft positions and orbital trajectories
%     
%     title(['Time = ',num2str(floor(t/3600)),' hours, ',num2str(percentage_seen),' % covered'],'fontsize',standard_font_size,'FontName','Times New Roman')
%     
%     %% Plot Observing Points Around Asteroid in 2D
%     subplot(2,4,3)
%     cla()
%     render_observed_points_2d(pos_points.*10^-3, point_index, n_spacecraft, 'above', 'color_array', color_array) % Show which points have been observed above equator
%     title('Above Equator','fontsize',standard_font_size,'FontName','Times New Roman')
%     set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
%     
%     subplot(2,4,4)
%     cla()
%     render_observed_points_2d(pos_points.*10^-3, point_index, n_spacecraft, 'below', 'color_array', color_array) % Show which points have been observed below equator
%     title('Below Equator','fontsize',standard_font_size,'FontName','Times New Roman')
%     set(gca, 'fontsize',standard_font_size,'FontName','Times New Roman')
%     
%     %% Plot Memory Use and Communication Topology
%     subplot(2,4,7)
%     plot_memory_comparison(sc_memory_use , communicating_sc_index, standard_comm_data_rate, delta_t, sc_max_memory, color_array, standard_font_size )
%     
%     subplot(2,4,8)
%     plot_communication_topology(sc_current_pos.*10^-3, carrier_current_pos.*10^-3, communicating_sc_index, ErosParameters.radius.*10^-3, color_array, standard_font_size)
%     
%     drawnow limitrate
%     
%     if (percentage_seen == 100) && (max(sc_memory_use) == 0)
%         break
%     end
% end
