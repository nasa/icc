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
%               Usage example of ICC comms optimization                   %
%                                                                         %
% Demonstrates the usage of the network communications optimization       %
% module. The module creates a simple scenario with two instrumented      %
% spacecraft and a relay spacecraft, and computes the optimal             %
% communication architecture to relay data to the carrier.                %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Load utilities that will be leveraged here
addpath(genpath('../utilities/'))
addpath(genpath('../small_body_dynamics/'))
addpath(genpath('../network_flow_communication_optimizer/'))
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath('../')
constants = initialize_SBDT();

% Create a gravity model
eros_sbdt = loadEros( constants, 1, 1, 3, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

% Create a swarm object
n_spacecraft = 4;
time_bounds = [0:300:86400*2];
sc_types = cell(n_spacecraft,1);
max_memory = ones(n_spacecraft,1)*1e10; %1TB

swarm = SpacecraftSwarm(time_bounds, sc_types, max_memory);

% Add four orbits for two sciencecrafts, one relay, and one carrier
sc_initial_state_array = zeros(n_spacecraft,6);
spacecraft_no = 1;
% SC 1
sc_location = [25*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel*sqrt(2)/2; sc_orbital_vel*sqrt(2)/2];
sc_initial_state_array(spacecraft_no,:) = [sc_location; sc_vel];
spacecraft_no = spacecraft_no+1;
% SC 2
sc_location = [25*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(spacecraft_no,:) = [sc_location; sc_vel];
spacecraft_no = spacecraft_no+1;
% Relay
sc_location = [50*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(spacecraft_no,:) = [sc_location; sc_vel];
spacecraft_no = spacecraft_no+1;
% Carrier
sc_location = [100*1e3;0;0];
sc_orbital_vel = sqrt(GM/norm(sc_location));
sc_vel = [0; sc_orbital_vel; 0];
sc_initial_state_array(spacecraft_no,:) = [sc_location; sc_vel];

% Integrate the spacecraft' orbits
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Add some science to the two sciencecraft
swarm.Observation.flow = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.flow(1:2,:) = 1e9; %1Gb/time step
swarm.Observation.priority = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.priority(1:2,:) = 1;

% Set up the communication model
bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

%bandwidth_model = @(x1,x2) min(bandwidth_parameters.reference_bandwidth * (bandwidth_parameters.reference_distance/norm(x2-x1,2))^2, bandwidth_parameters.max_bandwidth); 


spherical_asteroid_parameters.max_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;
spherical_asteroid_parameters.min_radius = ErosGravity.BodyModel.shape.maxRadius*1e3;

occlusion_test =  @(x1, x2) is_occluded(x1, x2, spherical_asteroid_parameters);
bandwidth_model = @(x1,x2) quadratic_comm_model(x1, x2, bandwidth_parameters, occlusion_test);


% Optimize communications
[swarm] = communication_optimizer(swarm, bandwidth_model);

% Plot the result

plot_communications(swarm, ErosGravity,'save_video',false, 'absolute', true);