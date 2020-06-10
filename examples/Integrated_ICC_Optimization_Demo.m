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

% Do you want to record video?
record_video = true;
videoname = ['ICC_integrated_',datestr(now,'yyyymmdd_HHMMSS'),'.mp4'];

% Do you want to save the output of the optimization in 42 format?
save_42_inputs = false;

rng default % Pseudo-random but repeatable scenario

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))
addpath(strcat(ROOT_PATH,'/relay_orbit_optimizer'))
addpath(strcat(ROOT_PATH,'/integrated_orbit_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_spacecraft = 7;  % Number of Spacecraft, counting the carrier

sc_types = cell(1,n_spacecraft);
for i_sc = 1:n_spacecraft
    sc_types{i_sc}  = randi([1,4]); % Indicies for instruments on board
end
carrier_index = n_spacecraft;
sc_types{carrier_index} = 0; % Mark the carrier so it will not be used in the Monte Carlo optimization
                            
                            
delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times


% Maximum time for the trust region integrated orbit optimizer, in seconds
max_optimization_time = 1800;

sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft); % 20 GB max memory for instrument spacecraft
sc_max_memory(1,carrier_index) = 8*10000*1e9; % Memory limit for carrier spacecraft

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
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

% In order to select a different gravity model, change the inputs to
% the loadEros function. See the help for assistance
eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
ErosModel = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt, constants);

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
if length(carrier_index)<1
    error("No carrier! This will not work")
end
carrier_initial_conditions = initialize_carrier_orbit(ErosModel);
Swarm.integrate_trajectory(carrier_index, ErosModel, carrier_initial_conditions);

% Initalize non-carrier trajectories
trial_initial_states = initialize_random_orbits(n_spacecraft, ErosModel);
for i_sc = setdiff(1:n_spacecraft, Swarm.get_indicies_of_type(0))
    Swarm.integrate_trajectory(i_sc, ErosModel, trial_initial_states(i_sc, :));
end


% % Get Sun Position - handled in constructor
% Swarm.sun_state_array = get_sun_state(Swarm.sample_times); 

%% Inside-the-reference-radius error
% This should turn the warning
% "Warning: HARMONIC_GRAVITY.DLL - Computing gravity inside the spherical harmonic reference radius"
% into an error. The plan is to catch the error inside communication_constraints.m
% To get this to work, ensure to patch SBDT as described in the README
warning('error', 'SBDT:harmonic_gravity:inside_radius')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     Integrated Orbit Optimization                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

[Swarm] = integrated_optimization(Swarm, ErosModel, bandwidth_parameters, max_optimization_time);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Show Combined Results                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Do you want the 3d plot to be in an absolute or relative frame?
absolute = true;

plot_coverage_and_communications_with_insets(Swarm, ErosModel,'absolute', absolute, 'record_video', record_video, 'video_name', videoname)

% Create 42 representation of the orbits
if save_42_inputs
    fortytwo_bridge(Swarm, ErosModel, "../utilities/42_bridge/defaults", strcat("42_EROS_ICC_ICC_",datestr(now,'yyyymmdd_HHMMSS')));    
end

cspice_kclear % This cleares the SPICE files from Matlab's memory