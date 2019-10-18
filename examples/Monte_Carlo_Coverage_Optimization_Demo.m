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

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))
addpath(strcat(ROOT_PATH,'/monte_carlo_coverage_optimizer'))


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set Parameters:
n_spacecraft = 2 ; % Number of Spacecraft, not counting the carrier

sc_types = cell(1,n_spacecraft);
sc_types{1} = 1;
sc_types{2} = 2;

delta_t = 10*60; % [s]; simulation time step
total_t = 1*24*60*60; % [s]; 1/2 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

n_trial_orbits = 10 ;
sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft-1); % 20 GB max memory for instrument spacecraft
sc_max_memory(1,n_spacecraft) = 8*10000*1e9; % Memory limit for carrier spacecraft

color_array = ['r', 'b', 'g', 'c', 'm'];

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

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Optimization                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimize Orbits and Observed Points with Monte Carlo
Swarm = monte_carlo_coverage_optimizer_main(ErosModel, Swarm, n_trial_orbits);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              Show Results                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Show Observed Points
h1=figure(1);
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');
initialize_spatial_plot_3d()
hold on 
render_asteroid_3d(ErosModel);
axis equal
axis([-1 1 -1 1 -1 1].*40)

for i_time = 1:Swarm.get_num_timesteps()
    try %#ok<TRYNC>
        for i_sc = 1:n_spacecraft
            if ~isempty(Swarm.Observation.observable_points{i_sc, i_time})
                h_op(i_sc) = render_these_points_3d(ErosModel, Swarm.Observation.observable_points{i_sc, i_time}, 'color', 'y', 'markersize', 3); %#ok<*SAGROW>
                render_these_points_3d(ErosModel, Swarm.Observation.observed_points(i_sc, i_time), 'color', color_array(i_sc), 'markersize', 6);
                h_line(i_sc) = plot3([Swarm.rel_trajectory_array(i_time,1,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),1)], ...
                    [Swarm.rel_trajectory_array(i_time,2,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),2)],...
                    [Swarm.rel_trajectory_array(i_time,3,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),3)],'--','color',color_array(i_sc),'linewidth',0.5);
            end
        end
        h_sc = render_spacecraft_3d(Swarm.rel_trajectory_array(1:i_time,1:3,:)./1000,'color', color_array, 'linewidth', 0.75);
        
        drawnow limitrate
        pause(0.125); delete(h_op); delete(h_sc); delete(h_line);
    end
end
h_sc = render_spacecraft_3d(Swarm.rel_trajectory_array(1:i_time,1:3,:)./1000,'color', color_array, 'showSC', false);
