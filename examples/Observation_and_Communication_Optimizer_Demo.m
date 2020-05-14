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
%        Usage example of Observation and Communication Optimizer         %
%                                                                         %
% Demonstrates usage of the integrated observation and relay optimization %
% module by finding the best observation points _and_ relays that         %
% maximize the amount of data collected and relayed to the carrier for a  %
% given set of orbits.                                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/network_flow_communication_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rng(0); % Get a consistent seed

n_spacecraft = 4;  % Number of Spacecraft, including the carrier

sc_types = {2, 3, 5, 0};  % Spectrometer, camera, radio science, carrier

sc_max_memory = ones(n_spacecraft,1)*1e12; %1TB

sc_max_memory(n_spacecraft) = 5*1e13; % 50 TB

delta_t = 10*60; % [s]; simulation time step
total_t = 0.5*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

color_array = ['k', 'r', 'b', 'g', 'c', 'm', 'y'];


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Initialize Eros Model                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

userModelsPath = strcat(SBDT_PATH,'/ExampleUserModels');
constantsModel = 1;
addpath(strcat(SBDT_PATH,'/Startup'));
constants = addSBDT(SBDT_PATH, userModelsPath, constantsModel);

% Create a gravity model
% In order to select a different gravity model, change the inputs to
% the loadEros function. See the help for assistance
[~, eros_sbdt] = evalc("loadEros( constants, 1, 1, 4, 3 );");
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Initialize Swarm Model                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Instantiate SpacecraftSwarm class for handling spacecraft data
Swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

% Add four orbits for two sciencecrafts, one relay, and one carrier
sc_initial_locations = {[25*1e3;0;0],[25*1e3;0;0],[50*1e3;0;0], [100*1e3;0;0]};
sc_initial_velocity_orientations = {[0; sqrt(2)/2; sqrt(2)/2;],[0;1;0],[0;1;0], [0;1;0]};

sc_initial_state_array = zeros(n_spacecraft,6);

for spacecraft_no = 1:n_spacecraft
    sc_location = sc_initial_locations{spacecraft_no};
    sc_vel_orientation = sc_initial_velocity_orientations{spacecraft_no};
    sc_orbital_vel = sqrt(GM/norm(sc_location));
    sc_vel = sc_vel_orientation*sc_orbital_vel;
    sc_initial_state_array(spacecraft_no,:) = [sc_location; sc_vel];
end

% Integrate the spacecraft' orbits
Swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Check for collisions
if Swarm.collision_with_asteroid(ErosGravity)
    fprintf('\nCollision with the asteroid!\n\n')
    percentage_seen = 0;
    return
end

% Set up the communication model
bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

spherical_asteroid_parameters.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;

occlusion_test =  @(x1, x2) is_occluded(x1, x2, spherical_asteroid_parameters);
bandwidth_model = @(x1,x2) quadratic_comm_model(x1, x2, bandwidth_parameters, occlusion_test);

data_scaling_factor = 1e6;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Find Best Observation Points for Given Orbits               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Swarm = observed_points_optimizer_main(ErosModel, Swarm, bandwidth_model);

[Swarm] = observation_and_communication_optimizer(ErosGravity, Swarm, bandwidth_model, data_scaling_factor);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Show Observed Points Simulation                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
absolute = false;

plot_coverage_and_communications_with_insets(Swarm, ErosGravity,'absolute', absolute, 'record_video', false)

cspice_kclear % This cleares the SPICE files from Matlab's memory