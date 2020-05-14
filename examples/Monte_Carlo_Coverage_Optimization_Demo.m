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
%           Usage example of Monte Carlo Coverage Optimizer               %
%                                                                         %
% Demonstrates the usage of the monte_carlo_coverage_optimizer module by  %
% finding the best orbits from a sequential Monte Carlo. Results are      %
% simulated using the visualization utilities.                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup_matlab.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(genpath(strcat(ROOT_PATH,'/visualization')))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 8; % Number of Spacecraft, counting the carrier

sc_types = cell(1,n_spacecraft);
for i_sc = 1:n_spacecraft
    sc_types{i_sc}  = randi([1,6]); % Indicies for instruments on board
end
carrier_index = n_spacecraft - 1;
sc_types{carrier_index} = 0; % Mark the carrier so it will not be used in the Monte Carlo optimization

delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1/2 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

n_trial_orbits = 10 ;

sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft); % 20 GB max memory for instrument spacecraft
sc_max_memory(1,carrier_index) = 8*10000*1e9; % Memory limit for carrier spacecraft

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
carrier_initial_conditions = initialize_carrier_orbit(ErosModel);
Swarm.integrate_trajectory(carrier_index, ErosModel, carrier_initial_conditions);

% % Get Sun Position - handled in constructor
% Swarm.sun_state_array = get_sun_state(Swarm.sample_times); 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Optimization                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimize Orbits and Observed Points with Monte Carlo
Swarm = monte_carlo_coverage_optimizer_main(ErosModel, Swarm, n_trial_orbits);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              Show Results                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot_coverage(Swarm, ErosModel,'absolute', true)

cspice_kclear % This cleares the SPICE files from Matlab's memory