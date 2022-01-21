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

COMPUTE_MILP = false;

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

scenarios_to_test = 501;


rng(0); % Get a consistent seed

n_spacecraft = 7;  % Number of Spacecraft, including the carrier

carrier_index = n_spacecraft;

% sc_types = {2, 3, 4, 0};  % Spectrometer, camera, radio science, carrier

sc_max_memory = 8*20*1e9.*ones(1,n_spacecraft); % 20 GB max memory for instrument spacecraft
sc_max_memory(1,carrier_index) = 8*10000*1e9; % Memory limit for carrier spacecraft

delta_t = 10*60; % [s]; simulation time step
total_t = 7*24*60*60; % [s]; 1 day, total time of simulation
time_vector = 0:delta_t:total_t; % sample times

color_array = ['k', 'r', 'b', 'g', 'c', 'm', 'y'];

mydatestring = datestr(datetime,"yyyymmdd_HHMMSS");

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

inner_solve_times_lp   = zeros(scenarios_to_test,1);
total_solve_times_lp   = zeros(scenarios_to_test,1);
inner_solve_times_lpt  = zeros(scenarios_to_test,1);
total_solve_times_lpt  = zeros(scenarios_to_test,1);
inner_solve_times_milp = zeros(scenarios_to_test,1);
total_solve_times_milp = zeros(scenarios_to_test,1);

goals_lp   = zeros(scenarios_to_test,1);
goals_lpt  = zeros(scenarios_to_test,1);
goals_milp = zeros(scenarios_to_test,1);

% Warm up parfor by starting the parallel pool
parfor i=1:20
    i;
end

for scenario_index = 1:scenarios_to_test


    sc_types = cell(1,n_spacecraft);
    for i_sc = 1:n_spacecraft
        sc_types{i_sc}  = randi([1,4]); % Indicies for instruments on board
    end
    sc_types{carrier_index} = 0; % Mark the carrier so it will not be used in the Monte Carlo optimization
    
    % Instantiate SpacecraftSwarm class for handling spacecraft data
    Swarm = SpacecraftSwarm(time_vector, sc_types, sc_max_memory);

    sc_initial_state_array = initialize_random_orbits(n_spacecraft, ErosGravity);
    carrier_initial_conditions = initialize_carrier_orbit(ErosGravity);
    sc_initial_state_array(carrier_index,:) = carrier_initial_conditions;    

    if scenario_index == 431 % Bad and for some reason we do not catch it
        continue
    end

    % Integrate the spacecraft' orbits
    Swarm.integrate_trajectories(ErosGravity, sc_initial_state_array);

    % Check for collisions
    if Swarm.collision_with_asteroid(ErosGravity)
        fprintf('\nCollision with the asteroid!\n\n')
        percentage_seen = 0;
        return
    end



    observation_and_communication_optimizer_options.verbose = true;
    observation_and_communication_optimizer_options.ilp = false;
    observation_and_communication_optimizer_options.truncate = false;

    % Swarm = observed_points_optimizer_main(ErosModel, Swarm, bandwidth_model);
    disp("LP, no truncation")
    lptic = tic;
    [Swarm_lp, goal_lp, problem_solve_time_lp] = observation_and_communication_optimizer(ErosGravity, Swarm, bandwidth_model, data_scaling_factor, observation_and_communication_optimizer_options);
    total_solve_times_lp(scenario_index) = toc(lptic);
    inner_solve_times_lp(scenario_index) = problem_solve_time_lp;
    goals_lp(scenario_index) = goal_lp;
    disp("LP truncated")
    observation_and_communication_optimizer_options.ilp = false;
    observation_and_communication_optimizer_options.truncate = true;
    lpttic = tic;
    [Swarm_lpt, goal_truncated, problem_solve_time_truncated] = observation_and_communication_optimizer(ErosGravity, Swarm, bandwidth_model, data_scaling_factor, observation_and_communication_optimizer_options);
    total_solve_times_lpt(scenario_index) = toc(lpttic);
    inner_solve_times_lpt(scenario_index) = problem_solve_time_truncated;
    goals_lpt(scenario_index) = goal_truncated;
    
    if COMPUTE_MILP
        disp("MILP")
        observation_and_communication_optimizer_options.ilp = true;
        observation_and_communication_optimizer_options.truncate = false;
        milptic = tic;
        [Swarm_milp, goal_milp, problem_solve_time_milp] = observation_and_communication_optimizer(ErosGravity, Swarm, bandwidth_model, data_scaling_factor, observation_and_communication_optimizer_options);
        total_solve_times_milp(scenario_index) = toc(milptic);
        inner_solve_times_milp(scenario_index) = problem_solve_time_milp;
        goals_milp(scenario_index) = goal_milp;
    else
        total_solve_times_milp(scenario_index) = NaN;
        inner_solve_times_milp(scenario_index) = NaN;
        goals_milp(scenario_index) = NaN;
    end
    save(strcat('benchmarks/inner_loop_intermediate_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring))
end

% total_solve_times_milp(431:432)=total_solve_times_milp(501:502)
% inner_solve_times_milp(431:432)=inner_solve_times_milp(501:502)
% goals_milp(431:432)=goals_milp(501:502)

% total_solve_times_lpt(431:432)=total_solve_times_lpt(501:502)
% inner_solve_times_lpt(431:432)=inner_solve_times_lpt(501:502)
% goals_lpt(431:432)=goals_lpt(501:502)

% total_solve_times_lp(431:432)=total_solve_times_lp(501:502)
% inner_solve_times_lp(431:432)=inner_solve_times_lp(501:502)
% goals_lp(431:432)=goals_lp(501:502)



% total_solve_times_milp=total_solve_times_milp(1:500);
% inner_solve_times_milp=inner_solve_times_milp(1:500);
% goals_milp=goals_milp(1:500);

% total_solve_times_lpt=total_solve_times_lpt(1:500);
% inner_solve_times_lpt=inner_solve_times_lpt(1:500);
% goals_lpt=goals_lpt(1:500);

% total_solve_times_lp=total_solve_times_lp(1:500);
% inner_solve_times_lp=inner_solve_times_lp(1:500);
% goals_lp=goals_lp(1:500);

save(strcat('benchmarks/inner_loop_final_',string(bandwidth_parameters.reference_bandwidth),"_",mydatestring))

cspice_kclear % This cleares the SPICE files from Matlab's memory

%% Plotting

% Ratio between the optimal costs of Problem(6)(LP relax-ation) and
% Problem(5)(MILP)
figure()
histogram(goals_lp./goals_milp)
xlabel("Ratio between relaxation cost and MILP cost")
ylabel("Frequency")
set(gca,'FontSize',16)
saveas(gcf,strcat('lp_to_milp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".eps"),'epsc')
saveas(gcf,strcat('lp_to_milp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".png"))

% Ratio between the optimal costs of the post-processed ver-sion of Problem(6)and Problem(5)
figure()
histogram(goals_lpt./goals_milp)
xlabel("Ratio between post-processed relaxation cost and MILP cost")
ylabel("Frequency")
set(gca,'FontSize',16)
saveas(gcf,strcat('lpt_to_milp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".eps"),'epsc')
saveas(gcf,strcat('lpt_to_milp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".png"))

figure()
h_lpt_lp = histogram(goals_lpt./goals_lp);
xlabel("Ratio between post-processed relaxation cost and relaxation cost")
ylabel("Frequency")
set(gca,'FontSize',16)
saveas(gcf,strcat('lpt_to_lp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".eps"),'epsc')
saveas(gcf,strcat('lpt_to_lp_ratio_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".png"))


% Time required to compute an optimal solution to Problem 6
figure()
histogram(inner_solve_times_lpt)
xline(mean(inner_solve_times_lpt), '--r','LineWidth',2)
xlabel("Time [s]")
ylabel("Frequency")
set(gca,'FontSize',16)
saveas(gcf,strcat('inner_solve_time_lp_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".eps"),'epsc')
saveas(gcf,strcat('inner_solve_time_lp_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".png"))
disp(strcat("Mean inner solve time: ", num2str(mean(inner_solve_times_lpt)), "s"))

% Time required to compute observability and bandwidths,solve Problem(6), and compute the gradient
figure()
histogram(total_solve_times_lpt)
xline(mean(total_solve_times_lpt), '--r','LineWidth',2)
xlabel("Time [s]")
ylabel("Frequency")
set(gca,'FontSize',16)
saveas(gcf,strcat('full_solve_time_lp_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".eps"),'epsc')
saveas(gcf,strcat('full_solve_time_lp_', string(bandwidth_parameters.reference_bandwidth), "_",string(scenario_index),"_",mydatestring, ".png"))
disp(strcat("Mean total solve time: ", num2str(mean(total_solve_times_lpt)), "s"))