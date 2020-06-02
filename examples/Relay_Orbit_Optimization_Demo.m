%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%            Usage example of the relay optimization framework.           % 
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%                                                                             %cd
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Load things
addpath(genpath('../utilities/'))
addpath(genpath('../small_body_dynamics/'))
addpath(genpath('../network_flow_communication_optimizer/'))
addpath(genpath('../relay_orbit_optimizer/'))
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath('../')
constants = initialize_SBDT();

eros_sbdt = loadEros( constants, 1, 1, 4, 3 );
ErosGravity = SphericalHarmonicsGravityIntegrator_SBDT(eros_sbdt);
GM = eros_sbdt.gravity.gm * 1e9;  % Convert to m from km

% Create a swarm object
n_spacecraft = 4;
time_bounds = [0:300:86400*2];
sc_types = cell(n_spacecraft,1);
for i_sc = 1:n_spacecraft
    sc_types{i_sc}  = randi([1,4]); % Indicies for instruments on board
end
sc_types{n_spacecraft} = 0; % Mark the carrier so it will not be used in the Monte Carlo optimization

max_memory = ones(n_spacecraft,1)*1e10; %1TB

swarm = SpacecraftSwarm(time_bounds, sc_types, max_memory);

% Orbits
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
% Integrate
swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

% Science parameters
swarm.Observation.flow = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.flow(1,:) = 1e9; %1Gb/time step
swarm.Observation.priority = zeros(n_spacecraft,length(time_bounds));
swarm.Observation.priority(1:2,:) = 1;

% Bandwidth params
bandwidth_parameters.reference_bandwidth = 250000;
bandwidth_parameters.reference_distance = 100000;
bandwidth_parameters.max_bandwidth = 100*1e6;

% Try optimizing comms right now for sanity - with incorrect bandwidth
% model
% [swarm] = communication_optimizer(swarm);
% plot_communications(swarm, ErosGravity,true)

relay_orbit_indices = [2];

max_optimization_time = 150;

% Call the optimizer
disp("Optimizing")
[swarm] = relay_optimization(swarm, ErosGravity, bandwidth_parameters, relay_orbit_indices, max_optimization_time);

% Plot the result
plot_communications(swarm, ErosGravity,true)

% save(strcat('relay_optimization_', datestr(datetime,"yyyymmdd_HHMMSS")))

cspice_kclear % This cleares the SPICE files from Matlab's memory