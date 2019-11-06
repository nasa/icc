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
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))
addpath(strcat(ROOT_PATH,'/relay_orbit_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_spacecraft = 4;  % Number of Spacecraft, counting the carrier

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
%                        Relay Orbit Optimization                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

[Swarm] = relay_optimization(Swarm, ErosModel, bandwidth_parameters, relay_orbit_indices, max_relay_optimization_time);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Show Combined Results                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Do you want the 3d plot to be in an absolute or relative frame?
absolute = true;

% Do you want to record video?
record_video = true;

color_array = rand(3,Swarm.get_num_spacecraft());

if record_video
    videoname = ['ICC_simulation_',datestr(now,'yyyymmdd_HHMMSS'),'.mp4'];
    writerObj = VideoWriter(videoname, 'MPEG-4');
    writerObj.FrameRate = 30;   % Default 30
    writerObj.Quality = 100;    % Default 75
    open(writerObj);
end



% Test 3d plot to get the axes extent
h1 = figure();
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');
initialize_spatial_plot_3d();
hold on 
axis equal
% Initial plot - just to get a sense of the size
plot_coverage_and_communications_frame(Swarm, ErosModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);
axis equal
three_d_plot_axes = axis();
clf;

for time_step = 1:length(Swarm.sample_times)
    subplot(2,4,[1 2 5 6]);
    if time_step == 1
        initialize_spatial_plot_3d();
        axis(three_d_plot_axes);
        axis equal
    end

    plot_handles = plot_coverage_and_communications_frame(Swarm, ErosModel, time_step, 'absolute', absolute,'color_array', color_array);
    
    subplot(2,4,3)
    render_observed_points_2d(ErosModel, Swarm, 'above', 'time_limits', [1, time_step],'color_array', color_array) % Show which points have been observed above equator
    
    subplot(2,4,4)
    render_observed_points_2d(ErosModel, Swarm, 'below', 'time_limits', [1, time_step],'color_array', color_array) % Show which points have been observed above equator
    
    subplot(2,4,7)
    plot_memory_comparison_2d(time_step, Swarm, 'semilogflag', true,'color_array', color_array);
    
    subplot(2,4,8)
    plot_communication_topology_2d(time_step, Swarm, ErosModel,color_array);
    
    drawnow limitrate
    pause(0.125);
    if record_video
        F = getframe(h1);
        writeVideo(writerObj,F);
    end
    
    for entry_ix = 1:length(plot_handles)
        if ~isempty(plot_handles{entry_ix})
            delete(plot_handles{entry_ix})
        end
    end
    
    
end

subplot(2,4,[1 2 5 6]);
plot_coverage_and_communications_frame(Swarm, ErosModel,length(Swarm.sample_times), 'absolute', absolute, 'figure_handle', h1, 'color_array', color_array);

if record_video
    close(writerObj);
end

