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
%               Usage example of Observed Points Optimizer                %
%                                                                         %
% Demonstrates usage of the observed_points_optimizer module by finding   %
% the best observation points accociated with randomly generated orbits.  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
addpath(strcat(ROOT_PATH,'/visualization'))
addpath(strcat(ROOT_PATH,'/observed_points_optimizer'))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   User Options: Flags and Parameters                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_spacecraft = 3;  % Number of Spacecraft, not counting the carrier

sc_types = cell(1,n_spacecraft);
for i_sc = 1:n_spacecraft
    sc_types{i_sc}  = randi([1,6]); % Indicies for instruments on board
end

sc_max_memory = zeros(1,n_spacecraft); % not used, but must be defined

delta_t = 10*60; % [s]; simulation time step
total_t = 0.5*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

color_array = ['k', 'r', 'b', 'g', 'c', 'm', 'y'];

flag_simulation = 1; % 1 to show simulation; 2 to show plot

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

% Initialize spacecraft states and integrate in absolute frame
sc_initial_state_array = initialize_random_orbits(n_spacecraft, ErosModel); % [m, m/s]
Swarm.integrate_trajectories(ErosModel, sc_initial_state_array, 'absolute');

% Check for collisions
if Swarm.collision_with_asteroid(ErosModel)
    fprintf('\nCollision with the asteroid!\n\n')
    percentage_seen = 0;
    return
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Find Best Observation Points for Given Orbits               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Swarm = observed_points_optimizer_main(ErosModel, Swarm);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Show Observed Points Simulation                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Show Observed Points
h1=figure(1);
set(h1,'Color',[1 1 1]);
set(h1,'units','normalized','outerposition',[0 0 1 1])
set(h1,'PaperPositionMode','auto');

initialize_spatial_plot_3d()
render_asteroid_3d(ErosModel);
axis equal
axis([-1 1 -1 1 -1 1].*40)

if flag_simulation==1
    for i_time = 1:Swarm.get_num_timesteps()
        hold on
        for i_sc = 1:n_spacecraft
            if ~isempty(Swarm.Observation.observable_points{i_sc, i_time}) && Swarm.Observation.observed_points(i_sc,i_time)~=0
                h_op(i_sc) = render_these_points_3d(ErosModel, Swarm.Observation.observable_points{i_sc, i_time}, 'color', 'y', 'markersize', 2); %#ok<*SAGROW>
                render_these_points_3d(ErosModel, Swarm.Observation.observed_points(i_sc, i_time), 'color', color_array(i_sc), 'markersize', 5);
                h_line(i_sc) = plot3([Swarm.rel_trajectory_array(i_time,1,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),1)], ...
                    [Swarm.rel_trajectory_array(i_time,2,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),2)],...
                    [Swarm.rel_trajectory_array(i_time,3,i_sc)./1000, ErosModel.BodyModel.shape.vertices(Swarm.Observation.observed_points(i_sc,i_time),3)],'--','color',color_array(i_sc),'linewidth',0.6);
            end
        end
        if i_time<Swarm.get_num_timesteps()
            h_sc = render_spacecraft_3d(Swarm.rel_trajectory_array(1:i_time,1:3,:)./1000,'color', color_array);
        end
        drawnow limitrate
        pause(0.125); delete(h_op); delete(h_sc); delete(h_line);
    end
    
elseif flag_simulation == 2
    render_observed_points_3d(ErosModel, Swarm);
    render_spacecraft_3d(Swarm.rel_trajectory_array./1000,'color', color_array);
end
